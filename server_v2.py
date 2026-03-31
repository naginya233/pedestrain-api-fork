#!/usr/bin/env python3
"""
Pedestrian Slow-Pass Flow Analysis Server v2
- Watches three folders: raw images / BEV images / scene graph JSONs
- Matches files by stem (000000.jpg <-> 000000_intersection.png <-> 000000_scene_graph.json)
- Serves real-time analysis via REST API
- Pure Python stdlib, no extra dependencies
Run: python3 server.py --images ./images --bev ./bev --graphs ./graphs [--port 8765]
"""

import json, sys, os, re, time, threading, argparse, mimetypes, base64
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from pathlib import Path

# ─────────────────────────────────────────────────────────────────────────────
# SPI Engine (same logic as v1)
# ─────────────────────────────────────────────────────────────────────────────

VEHICLE_WEIGHT = {
    "TRUCK": 1.4, "BUS": 1.4, "CAR": 1.0,
    "VAN": 1.1, "MOTORCYCLIST": 0.6, "CYCLIST": 0.4, "PEDESTRIAN": 0.0,
}
LEVEL_COLOR = {"SAFE":"#1D9E75","CAUTION":"#0F6E56","SLOW":"#BA7517","STOP":"#E24B4A"}
LEVEL_BG    = {"SAFE":"#EAF3DE","CAUTION":"#E1F5EE","SLOW":"#FAEEDA","STOP":"#FCEBEB"}

def layer1(map_triples):
    cw = {}
    for t in map_triples:
        if t.get("object_type") == "CROSSWALK" and t.get("state") == "inside":
            cid = t["object"]; vt = t.get("subject_type","CAR")
            r = t.get("inter_ratio", 0.0); w = VEHICLE_WEIGHT.get(vt, 1.0)
            cw.setdefault(cid, {"occupants":[], "raw":0.0})
            sc = r * w
            cw[cid]["occupants"].append({"id":t["subject"],"type":vt,"inter_ratio":round(r,3),"weight":w,"contribution":round(sc,3)})
            cw[cid]["raw"] += sc
    total = 0.0; details = []
    for cid, d in cw.items():
        norm = min(d["raw"]/1.4,1.0)*100; total += norm
        details.append({"crosswalk_id":cid,"occupants":d["occupants"],"raw_score":round(d["raw"],3),"normalized_score":round(norm,1)})
    score = (total/len(cw)) if cw else 0.0
    
    for t in map_triples:
        if t.get("object_type") == "CROSSWALK" and t.get("state") == "inside":
            vt = t.get("subject_type", "CAR")
            if vt == "PEDESTRIAN":   # ← 新增：跳过行人
                continue

    return {"score":round(min(score,100),1),"crosswalk_count":len(cw),"details":details}

   

def layer2(map_triples):
    vehicles = []
    for t in map_triples:
        meta = t.get("object_meta",{})
        if meta.get("is_intersection") and t.get("inter_ratio",0)>0.5 and t.get("subject_type")!="PEDESTRIAN":
            vehicles.append({"id":t["subject"],"type":t.get("subject_type"),"inter_ratio":round(t.get("inter_ratio",0),3),"lane_id":t.get("object","")})
    count = len(vehicles)
    return {"score":round(min(count/6.0*100,100),1),"vehicle_count":count,"vehicles":vehicles}

def layer3(map_triples, obj_triples):
    cw_veh = {t["subject"] for t in map_triples if t.get("object_type")=="CROSSWALK"}
    chains = []; conflict = 0
    for t in obj_triples:
        if t.get("relation")=="following":
            risk = t["object"] in cw_veh
            if risk: conflict += 35
            chains.append({"follower":t["subject"],"leader":t["object"],"risk":risk})
    total = min(conflict + min(len(obj_triples)*10,40), 100)
    return {"score":round(total,1),"following_chains":chains,"crosswalk_blocking_vehicles":list(cw_veh),"chain_count":len(chains)}

def classify(spi):
    if spi < 25: return "SAFE"
    if spi < 50: return "CAUTION"
    if spi < 75: return "SLOW"
    return "STOP"

def advisory(level, l1, l2, l3):
    base = {"SAFE":"行人可正常通过斑马线，当前路口车辆稀少且无车辆压线。",
            "CAUTION":"行人请注意观察，路口存在少量车辆，建议确认车辆停稳后通过。",
            "SLOW":"行人需缓行通过，斑马线存在车辆占用或路口排队明显，请确认安全后谨慎通行。",
            "STOP":"行人请勿通过！斑马线被车辆占用或路口严重拥堵，等待路况好转。"}[level]
  # advisory 函数里，只报告真正的障碍车辆
    extras = [
        f"车辆{o['id']}({o['type']})占据斑马线{o['inter_ratio']*100:.0f}%"
        for d in l1.get("details",[])
        for o in d.get("occupants",[])
        if o["inter_ratio"] > 0.3
        and o["type"] not in ("PEDESTRIAN", "CYCLIST")  # ← 新增过滤
    ]
    risky = [c for c in l3.get("following_chains",[]) if c.get("risk")]
    if risky: extras.append(f"{len(risky)}辆车排队驶向斑马线区域")
    if extras: base += " 风险：" + "；".join(extras[:3]) + "。"
    return base

def analyze(sg, weights=(0.5,0.3,0.2)):
    mt = sg.get("object_map_triples",[])
    ot = sg.get("object_object_triples",[])
    l1r = layer1(mt); l2r = layer2(mt); l3r = layer3(mt, ot)
    spi = round(min(l1r["score"]*weights[0]+l2r["score"]*weights[1]+l3r["score"]*weights[2],100),1)
    lv  = classify(spi)
    return {"image_id":sg.get("image_id","?"),"spi":spi,"level":lv,
            "level_color":LEVEL_COLOR[lv],"level_bg":LEVEL_BG[lv],
            "advisory":advisory(lv,l1r,l2r,l3r),
            "layers":{"layer1_crosswalk_occupancy":l1r,"layer2_intersection_density":l2r,"layer3_vehicle_behavior":l3r},
            "weights":{"layer1":weights[0],"layer2":weights[1],"layer3":weights[2]}}

# ─────────────────────────────────────────────────────────────────────────────
# Folder Watcher — scans three directories and matches files by stem ID
# ─────────────────────────────────────────────────────────────────────────────

IMG_EXT  = {".jpg",".jpeg",".png",".webp",".bmp"}
BEV_EXT  = {".jpg",".jpeg",".png",".webp",".bmp"}
JSON_EXT = {".json"}

def extract_id(stem: str) -> str:
    """Extract numeric/alphanumeric ID from filename stem.
    000000_intersection -> 000000
    000000_scene_graph  -> 000000
    000000              -> 000000
    frame_001           -> 001
    """
    # Remove common suffixes
    for suffix in ["_scene_graph","_intersection","_bev","_seg","_map"]:
        stem = stem.replace(suffix, "")
    # Return cleaned stem
    return stem.strip("_- ")

class FolderWatcher:
    def __init__(self, images_dir, bev_dir, graphs_dir, interval=2.0):
        self.dirs = {
            "images": Path(images_dir) if images_dir else None,
            "bev":    Path(bev_dir)    if bev_dir    else None,
            "graphs": Path(graphs_dir) if graphs_dir else None,
        }
        self.interval = interval
        self._lock  = threading.Lock()
        self._cache = {}        # id -> {image, bev, graph, result, mtime}
        self._mtimes = {}       # path -> mtime
        self._running = False
        self._thread  = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[Watcher] Started, scanning every", self.interval, "s")

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            try:
                self._scan()
            except Exception as e:
                print("[Watcher] Scan error:", e)
            time.sleep(self.interval)

    def _scan(self):
        # Collect all files per category
        images = {}; bevs = {}; graphs = {}

        def collect(d, ext_set, dest):
            if d and d.exists():
                for f in d.iterdir():
                    if f.suffix.lower() in ext_set:
                        fid = extract_id(f.stem)
                        dest[fid] = f

        collect(self.dirs["images"], IMG_EXT,  images)
        collect(self.dirs["bev"],    BEV_EXT,  bevs)
        collect(self.dirs["graphs"], JSON_EXT, graphs)

        all_ids = set(images) | set(bevs) | set(graphs)

        with self._lock:
            for fid in all_ids:
                img_path   = images.get(fid)
                bev_path   = bevs.get(fid)
                graph_path = graphs.get(fid)

                # Check if anything changed
                changed = False
                for p in [img_path, bev_path, graph_path]:
                    if p is None: continue
                    mt = p.stat().st_mtime
                    if self._mtimes.get(str(p)) != mt:
                        self._mtimes[str(p)] = mt
                        changed = True

                if fid not in self._cache or changed:
                    entry = {
                        "id": fid,
                        "image_path": str(img_path)   if img_path   else None,
                        "bev_path":   str(bev_path)   if bev_path   else None,
                        "graph_path": str(graph_path) if graph_path else None,
                        "result": None,
                        "error":  None,
                        "ts": time.time(),
                    }
                    # Run analysis if graph exists
                    if graph_path:
                        try:
                            with open(graph_path, encoding="utf-8") as f:
                                sg = json.load(f)
                            if "image_id" not in sg:
                                sg["image_id"] = fid
                            entry["result"] = analyze(sg)
                        except Exception as e:
                            entry["error"] = str(e)
                    self._cache[fid] = entry
                    if changed:
                        print(f"[Watcher] Updated: {fid}")

            # Remove stale entries no longer in any folder
            stale = [k for k in self._cache if k not in all_ids]
            for k in stale:
                del self._cache[k]
                print(f"[Watcher] Removed: {k}")

    def get_all(self):
        with self._lock:
            return sorted(self._cache.values(), key=lambda x: x["id"])

    def get_one(self, fid):
        with self._lock:
            return self._cache.get(fid)

    def set_dirs(self, images_dir=None, bev_dir=None, graphs_dir=None):
        with self._lock:
            if images_dir is not None: self.dirs["images"] = Path(images_dir) if images_dir else None
            if bev_dir    is not None: self.dirs["bev"]    = Path(bev_dir)    if bev_dir    else None
            if graphs_dir is not None: self.dirs["graphs"] = Path(graphs_dir) if graphs_dir else None
            self._cache.clear(); self._mtimes.clear()
        print("[Watcher] Dirs updated, cache cleared")

    def status(self):
        with self._lock:
            return {
                "images_dir": str(self.dirs["images"]) if self.dirs["images"] else None,
                "bev_dir":    str(self.dirs["bev"])    if self.dirs["bev"]    else None,
                "graphs_dir": str(self.dirs["graphs"]) if self.dirs["graphs"] else None,
                "total_ids":  len(self._cache),
                "analyzed":   sum(1 for v in self._cache.values() if v["result"]),
                "errors":     sum(1 for v in self._cache.values() if v["error"]),
            }

# ─────────────────────────────────────────────────────────────────────────────
# HTTP Handler
# ─────────────────────────────────────────────────────────────────────────────

watcher: FolderWatcher = None  # set in main

def image_to_data_url(path):
    if not path or not Path(path).exists():
        return None
    mt, _ = mimetypes.guess_type(path)
    mt = mt or "image/jpeg"
    with open(path, "rb") as f:
        data = base64.b64encode(f.read()).decode()
    return f"data:{mt};base64,{data}"

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print(f"[{self.address_string()}] {fmt%args}")

    def cors(self):
        self.send_header("Access-Control-Allow-Origin","*")
        self.send_header("Access-Control-Allow-Headers","Content-Type")
        self.send_header("Access-Control-Allow-Methods","GET,POST,OPTIONS")

    def send_json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type","application/json; charset=utf-8")
        self.send_header("Content-Length",str(len(body)))
        self.cors()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204); self.cors(); self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path   = parsed.path
        qs     = parse_qs(parsed.query)

        if path == "/health":
            self.send_json({"status":"ok","version":"2.0"})

        elif path == "/status":
            self.send_json(watcher.status())

        elif path == "/list":
            items = []
            for entry in watcher.get_all():
                r = entry.get("result")
                items.append({
                    "id":       entry["id"],
                    "has_image": entry["image_path"] is not None,
                    "has_bev":   entry["bev_path"]   is not None,
                    "has_graph": entry["graph_path"]  is not None,
                    "spi":       r["spi"]   if r else None,
                    "level":     r["level"] if r else None,
                    "level_color": r["level_color"] if r else None,
                    "error":     entry.get("error"),
                    "ts":        entry["ts"],
                })
            self.send_json({"items": items, "total": len(items)})

        elif path == "/detail":
            fid = qs.get("id", [None])[0]
            if not fid:
                self.send_json({"error":"Missing ?id="}, 400); return
            entry = watcher.get_one(fid)
            if not entry:
                self.send_json({"error":f"ID '{fid}' not found"}, 404); return
            # Include images as data URLs (inline base64)
            include_img = qs.get("img",["1"])[0] == "1"
            resp = dict(entry)
            if include_img:
                resp["image_data"] = image_to_data_url(entry["image_path"])
                resp["bev_data"]   = image_to_data_url(entry["bev_path"])
            # Don't leak full paths to frontend, just show filenames
            resp["image_file"] = Path(entry["image_path"]).name if entry["image_path"] else None
            resp["bev_file"]   = Path(entry["bev_path"]).name   if entry["bev_path"]   else None
            resp["graph_file"] = Path(entry["graph_path"]).name if entry["graph_path"] else None
            self.send_json(resp)

        else:
            self.send_json({"error":"Not found","endpoints":[
                "GET  /health", "GET  /status",
                "GET  /list", "GET  /detail?id=<id>&img=1",
                "POST /config", "POST /analyze",
            ]}, 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length",0))
        body   = json.loads(self.rfile.read(length).decode()) if length else {}
        path   = urlparse(self.path).path

        if path == "/config":
            # Update folder paths at runtime
            watcher.set_dirs(
                images_dir = body.get("images_dir"),
                bev_dir    = body.get("bev_dir"),
                graphs_dir = body.get("graphs_dir"),
            )
            self.send_json({"ok": True, "status": watcher.status()})

        elif path == "/analyze":
            # One-off analysis (no file needed, pass scene_graph directly)
            sg = body.get("scene_graph")
            if not sg:
                self.send_json({"error":"Missing scene_graph"},400); return
            w = body.get("weights", [0.5, 0.3, 0.2])
            self.send_json(analyze(sg, tuple(w)))

        else:
            self.send_json({"error":"Not found"}, 404)

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Pedestrian SPI Server v2")
    ap.add_argument("--images",  default=None, help="Path to raw images folder")
    ap.add_argument("--bev",     default=None, help="Path to BEV images folder")
    ap.add_argument("--graphs",  default=None, help="Path to scene graph JSON folder")
    ap.add_argument("--port",    default=8765, type=int)
    ap.add_argument("--interval",default=2.0,  type=float, help="Folder scan interval (seconds)")
    args = ap.parse_args()

    watcher = FolderWatcher(args.images, args.bev, args.graphs, args.interval)
    watcher.start()

    # Do one immediate scan before accepting requests
    time.sleep(0.5)

    server = HTTPServer(("0.0.0.0", args.port), Handler)
    print(f"\n Pedestrian SPI Engine v2  →  http://localhost:{args.port}")
    print(f"   Images dir : {args.images or '(not set)'}")
    print(f"   BEV dir    : {args.bev    or '(not set)'}")
    print(f"   Graphs dir : {args.graphs or '(not set)'}")
    print(f"   Scan every : {args.interval}s\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
        watcher.stop()
