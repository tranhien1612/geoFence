import argparse, os, time, json, math, threading, queue
from datetime import datetime
from shapely.geometry import shape, Point
from typing import List, Tuple, Dict, Any, Optional

class Logger:
    def __init__(self, folder_path = "/home/ubuntu/src/geoFence/log"):
        self.enable_log = True
        self.folder_path = folder_path
        self.log_file = folder_path + f"/nfz_monitor_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.log_queue = queue.Queue()
        self.log_thread = threading.Thread(target=self._log_worker, daemon=True)
        self.log_thread.start()

        
    def _log_worker(self):
        """Background thread: write log entries from queue to file."""
        while True:
            try:
                log_entry = self.log_queue.get()
                if log_entry is None:  # Stop signal
                    break
                with open(self.log_file, "a") as f:
                    f.write(log_entry + "\n")
            except Exception as e:
                print(f"[LOG ERROR] {e}")
                
    def write(self, message: str):
        """Log message to console and file."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        print(log_entry)
        if self.enable_log:
            self.log_queue.put(log_entry)

def deg_to_m_coeff(lat_deg: float) -> Tuple[float, float]:
    """Return (meters_per_deg_lat, meters_per_deg_lon) at the given latitude."""
    meters_per_deg_lat = 111_132.92 - 559.82 * math.cos(2*math.radians(lat_deg)) + 1.175 * math.cos(4*math.radians(lat_deg))
    meters_per_deg_lon = 111_412.84 * math.cos(math.radians(lat_deg)) - 93.5 * math.cos(3*math.radians(lat_deg))
    meters_per_deg_lon = max(1.0, meters_per_deg_lon)
    return meters_per_deg_lat, meters_per_deg_lon

def to_local_xy(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Approximate local x/y meters from (lat,lon) relative to origin (lat0,lon0)."""
    m_per_deg_lat, m_per_deg_lon = deg_to_m_coeff(lat0)
    dx = (lon - lon0) * m_per_deg_lon
    dy = (lat - lat0) * m_per_deg_lat
    return dx, dy

def point_in_poly_xy(px: float, py: float, ring_xy: List[Tuple[float, float]]) -> bool:
    """Ray-casting for a single ring in XY meters."""
    inside = False
    j = len(ring_xy) - 1
    for i in range(len(ring_xy)):
        xi, yi = ring_xy[i]
        xj, yj = ring_xy[j]
        if ((yi > py) != (yj > py)):
            x_int = (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
            if px < x_int:
                inside = not inside
        j = i
    return inside

def dist_point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Distance from point P to segment AB in meters."""
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    seg_len2 = vx*vx + vy*vy
    if seg_len2 <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx*vx + wy*vy) / seg_len2))
    cx, cy = ax + t*vx, ay + t*vy
    return math.hypot(px - cx, py - cy)

def polygon_distance_and_inside(lat0: float, lon0: float, lat: float, lon: float, rings_ll: List[List[Tuple[float,float]]]) -> Tuple[float, bool]:
    """Compute minimum distance (meters) from point to a Polygon with holes and inside status."""
    rings_xy: List[List[Tuple[float,float]]] = []
    for ring in rings_ll:
        if len(ring) >= 1 and ring[0] != ring[-1]:
            ring = ring + [ring[0]]
        rings_xy.append([to_local_xy(lat0, lon0, lat_i, lon_i) for (lon_i, lat_i) in ring])

    px, py = to_local_xy(lat0, lon0, lat, lon)

    inside_outer = point_in_poly_xy(px, py, rings_xy[0])
    inside_hole = any(point_in_poly_xy(px, py, hole) for hole in rings_xy[1:]) if len(rings_xy) > 1 else False
    inside = inside_outer and not inside_hole

    dmin = float("inf")
    for ring in rings_xy:
        for i in range(len(ring) - 1):
            ax, ay = ring[i]
            bx, by = ring[i+1]
            d = dist_point_to_segment(px, py, ax, ay, bx, by)
            if d < dmin:
                dmin = d
    return dmin, inside


class GeoFence:
    def __init__(self, geo_path = "/home/ubuntu/src/geoFence/geoData/"):
        self.geo_path = geo_path
        self.geo_country_path = self.geo_path + "countries/"
        self.geo_fence_path = self.geo_path + "fence/"

        self.nfz_polygons = None
        self.country_name = None
        # self.log = Logger()

    def check_nfz_proximity(self, lat: float, lon: float) -> Tuple[float, bool, str]:
        """Check proximity to NFZ. Returns (signed_distance, inside_any, nearest_name)."""
        if not self.nfz_polygons:
            return float("inf"), False, "No NFZ"
        
        min_edge = float("inf")
        nearest_name = "Unknown"
        inside_any = False
        
        for feat in self.nfz_polygons:
            d_edge, inside = polygon_distance_and_inside(lat, lon, lat, lon, feat["rings"])
            if inside and d_edge < min_edge:
                inside_any = True
                min_edge = d_edge
                nearest_name = feat["name"]
            elif not inside and d_edge < min_edge:
                min_edge = d_edge
                nearest_name = feat["name"]
        
        signed_distance = -min_edge if inside_any else min_edge
        return signed_distance, inside_any, nearest_name

    def load_geojson_polygons(self, path: str) -> List[Dict[str, Any]]:
        """Load GeoJSON polygons."""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        features: List[Dict[str, Any]] = []
        feats = data["features"] if data.get("type") == "FeatureCollection" else data.get("features", [])
        for feat in feats:
            geom = feat.get("geometry", {})
            gtype = geom.get("type")
            name = (feat.get("properties") or {}).get("name") or (feat.get("id") or "unnamed")
            coords = geom.get("coordinates", [])

            if gtype == "Polygon":
                rings = []
                for ring in coords:
                    rings.append([(float(lon), float(lat)) for lon, lat in ring])
                features.append({"name": name, "rings": rings})
            elif gtype == "MultiPolygon":
                for poly in coords:
                    rings = []
                    for ring in poly:
                        rings.append([(float(lon), float(lat)) for lon, lat in ring])
                    features.append({"name": name, "rings": rings})
        return features

    def get_country(self, folder_path: str, lat: float, lon: float) -> str:
        if lat is not None and lon is not None and folder_path is not None:
            point = Point(lon, lat)
            for filename in os.listdir(folder_path):
                if filename.endswith(".json") or filename.endswith(".geojson"):
                    filepath = os.path.join(folder_path, filename)
                    with open(filepath, "r", encoding="utf-8") as f:
                        data = json.load(f)

                    if "features" in data:
                        for feature in data["features"]:
                            geom = shape(feature["geometry"])
                            if geom.contains(point):
                                country = os.path.splitext(filename)[0]
                                return country
        return None

    def load_geojson(self, lat, lon):
        if self.nfz_polygons is None:
            if self.country_name is None:
                self.country_name = self.get_country(self.geo_country_path, lat, lon)
                # print(f"Position ({lat}, {lon}) from {self.country_name}")
                # self.log(f"Position ({self.current_lat}, {self.current_lon}) from {self.country_name}")
            else:
                geojson_path = self.geo_fence_path + self.country_name + ".geojson"
                self.nfz_polygons = self.load_geojson_polygons(geojson_path)
                # print(f"Loaded {len(self.nfz_polygons)} NFZ polygons from {self.country_name}.geojson")
                # self.log(f"Loaded {len(self.nfz_polygons)} NFZ polygons from {self.country_name}.geojson")






