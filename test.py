import argparse, os, time, json, math, threading, queue
from pymavlink import mavutil
from datetime import datetime
from shapely.geometry import shape, Point
from typing import List, Tuple, Dict, Any, Optional


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

def load_geojson_polygons(path: str) -> List[Dict[str, Any]]:
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

def get_country(folder_path: str, lat: float, lon: float) -> str:
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

class NFZMonitor:
    def __init__(self, log_path: str, geo_path: str, connection_string: str, warning_distance: float = 500.0, break_distance: float = 100.0):
        self.log_path = log_path

        self.geo_path = geo_path
        self.fence_path = geo_path + "fence/"
        self.country_path = geo_path + "countries/"

        self.connection_string = connection_string
        self.warning_distance = warning_distance  # meters
        self.break_distance = break_distance      # meters

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        self.country_name = None
        self.nfz_polygons = None

        # State tracking
        self.last_warning_time = 0
        self.last_break_time = 0
        self.warning_cooldown = 10.0  # seconds
        self.break_cooldown = 5.0     # seconds
        self.monitoring = False

        # Log
        self.enable_log = True
        self.log_file = log_path + f"nfz_monitor_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.log_queue = queue.Queue()
        self.log_thread = threading.Thread(target=self._log_worker, daemon=True)
        self.log_thread.start()

        # Flight mode tracking
        self.current_flight_mode = None
        self.original_flight_mode = None
        self.rtl_triggered = False
        self.mode_restore_distance = break_distance * 2.0  # Restore mode when 2x break distance away
        self.last_mode_request_time = 0
        self.mode_request_cooldown = 2.0  # seconds between mode requests

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
                
    def log(self, message: str):
        """Log message to console and file."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        print(log_entry)

        if self.enable_log:
            self.log_queue.put(log_entry)

    def load_geojson(self):
        if self.nfz_polygons is None:
            if self.country_name is None:
                self.country_name = get_country(self.country_path, self.current_lat, self.current_lon)
                self.log(f"Position ({self.current_lat}, {self.current_lon}) from {self.country_name}")
            else:
                geojson_path = self.fence_path + self.country_name + ".geojson"
                self.nfz_polygons = load_geojson_polygons(geojson_path)
                self.log(f"Loaded {len(self.nfz_polygons)} NFZ polygons from {self.country_name}.geojson")

    def get_flight_mode_name(self, mode_number):
        """Convert flight mode number to readable name for ArduPilot."""
        ardupilot_modes = {
            0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
            5: "LOITER", 6: "RTL", 7: "CIRCLE", 8: "POSITION", 9: "LAND",
            10: "OF_LOITER", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
            15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
            19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
            22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID",
            26: "AUTOROTATE", 27: "AUTO_RTL"
        }
        return ardupilot_modes.get(mode_number, f"UNKNOWN_{mode_number}")

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
    
    def restore_original_mode(self):
        """Restore the original flight mode when safe distance is reached."""
        if self.original_flight_mode is None:
            self.log("No original flight mode to restore")
            return False
            
        try:
            mode_name = self.get_flight_mode_name(self.original_flight_mode)
            self.log(f"Restoring original flight mode: {mode_name} (mode {self.original_flight_mode})")
            
            # Send mode change command
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.original_flight_mode
            )
            
            # Backup command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # Confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.original_flight_mode,
                0, 0, 0, 0, 0
            )
            
            self.rtl_triggered = False
            return True
            
        except Exception as e:
            self.log(f"Failed to restore flight mode: {e}")
            return False
    
    def connect_mavlink(self):
        """Establish MAVLink connection."""
        try:
            self.log(f"Connecting to MAVLink: {self.connection_string}")
            
            # Handle serial device connections with explicit baud rate
            if self.connection_string.startswith('/dev/') or self.connection_string.startswith('COM'):
                # For serial devices, ensure baud rate is specified
                if ':' not in self.connection_string:
                    self.connection_string += ':57600'  # Default baud rate
                    self.log(f"Added default baud rate: {self.connection_string}")
                
                # Parse serial device and baud rate
                parts = self.connection_string.split(':')
                device = parts[0]
                baud = int(parts[1]) if len(parts) > 1 else 57600
                self.log(f"Connecting to serial device: {device} at {baud} baud")
                self.master = mavutil.mavlink_connection(device, baud=baud)
            else:
                # UDP or other connection types
                self.master = mavutil.mavlink_connection(self.connection_string)
            
            # Wait for heartbeat with timeout
            self.log("Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)
            self.log(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}")
            
            # Request GPS data stream
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,  # 10 Hz
                1    # Enable
            )
            
            # Request flight mode and status updates
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                2,   # 2 Hz
                1    # Enable
            )
            
            return True
            
        except Exception as e:
            self.log(f"Failed to connect to MAVLink: {e}")
            return False

    def heartbeat_handle(self):
        try:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg is not None:
                custom_mode = msg.custom_mode
                mode_name = self.get_flight_mode_name(custom_mode)

                if self.current_flight_mode != custom_mode:
                    self.log(f"Flight mode changed: {mode_name} (mode {custom_mode})")
                    self.current_flight_mode = custom_mode

                if not self.rtl_triggered and custom_mode != 6:  # 6 = RTL
                    self.original_flight_mode = custom_mode

        except Exception as e:
            self.log(f"Error updating flight mode: {e}")    

    def gps_handle(self):
        try:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg is not None:
                self.current_lat = msg.lat / 1e7
                self.current_lon = msg.lon / 1e7
                self.current_alt = msg.relative_alt / 1000.0  # mm to meters
                self.log(f"Status: Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}, Alt={self.current_alt:.1f}m")
                self.load_geojson()

                # Check NFZ proximity
                signed_distance, inside_nfz, nfz_name = self.check_nfz_proximity(self.current_lat, self.current_lon)
                distance = abs(signed_distance)
                
                current_time = time.time()
                
                # Check if inside NFZ or too close -> RTL
                if inside_nfz or distance <= self.break_distance:
                    if current_time - self.last_break_time > self.break_cooldown:
                        self.log(f"CRITICAL: {'Inside' if inside_nfz else 'Too close to'} NFZ '{nfz_name}' - Distance: {signed_distance:.1f}m")
                        self.send_break_command()
                        self.last_break_time = current_time
                
                # Check if we should restore original mode (far enough from NFZ and RTL was triggered)
                elif self.rtl_triggered and distance > self.mode_restore_distance:
                    self.log(f"Safe distance reached ({distance:.1f}m > {self.mode_restore_distance:.1f}m), restoring original flight mode")
                    self.restore_original_mode()
                
                # Check if within warning distance
                elif distance <= self.warning_distance:
                    if current_time - self.last_warning_time > self.warning_cooldown:
                        self.send_warning(distance, nfz_name)
                        self.last_warning_time = current_time
                
                # Periodic status update (every 5 seconds)
                if hasattr(self, 'last_status_time'):
                    if current_time - self.last_status_time > 5.0:
                        mode_info = f", Mode={self.get_flight_mode_name(self.current_flight_mode) if self.current_flight_mode else 'Unknown'}"
                        rtl_info = f", RTL_triggered={self.rtl_triggered}" if self.rtl_triggered else ""
                        self.log(f"Status: Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}, Alt={self.current_alt:.1f}m, NFZ_dist={signed_distance:.1f}m ({nfz_name}){mode_info}{rtl_info}")
                        self.last_status_time = current_time
                else:
                    self.last_status_time = current_time

        except Exception as e:
            self.log(f"Error processing GPS data: {e}")

    def process_data(self):
        try:
            self.heartbeat_handle()
            self.gps_handle()
       
        except Exception as e:
            self.log(f"Error updating flight mode: {e}")
    
    # Message
    def send_break_command(self):
        """Send BREAK command to drone - RTL (Return To Launch) mode."""
        try:
            # Store current mode as original if not already stored
            if self.original_flight_mode is None and self.current_flight_mode is not None:
                self.original_flight_mode = self.current_flight_mode
                mode_name = self.get_flight_mode_name(self.current_flight_mode)
                self.log(f"Storing original flight mode: {mode_name} (mode {self.current_flight_mode})")
            
            # Method 1: Send RTL command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # RTL Command
                0,  # Confirmation
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0   # Empty
            )
            
            # Method 2: Set flight mode to RTL
            # Mode numbers: ArduPilot RTL=6, PX4 RTL=5
            rtl_mode = 6  # ArduPilot RTL mode
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                rtl_mode
            )
            
            # Method 3: Send MAV_CMD_DO_SET_MODE command as backup
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # Confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Mode
                rtl_mode,  # Custom mode (RTL)
                0,  # Custom submode
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0   # Empty
            )
            
            self.rtl_triggered = True
            self.log("BREAK COMMAND SENT - Drone commanded to RTL (Return To Launch)")
            return True
            
        except Exception as e:
            self.log(f"Failed to send RTL command: {e}")
            return False

    def send_warning(self, distance: float, nfz_name: str):
        """Send warning message (could be extended to send to GCS)."""
        self.log(f"WARNING: Approaching NFZ '{nfz_name}' - Distance: {distance:.1f}m")
        
        # Could send MAVLink text message to Ground Control Station
        try:
            warning_msg = f"WARNING: NFZ {distance:.0f}m"
            self.master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_WARNING,
                warning_msg.encode()[:50]  # Max 50 chars
            )
        except Exception as e:
            self.log(f"Failed to send warning message: {e}")
    ##

    def start_monitoring(self):
        """Start the NFZ monitoring loop."""
        if not self.connect_mavlink():
            return False
        self.monitoring = True

        try:
            while self.monitoring:
                self.process_data()
                time.sleep(0.1)  # 10Hz update rate
                
        except KeyboardInterrupt:
            self.log("Monitoring stopped by user")
        except Exception as e:
            self.log(f"Monitoring error: {e}")
        finally:
            self.stop_monitoring()
    
    def stop_monitoring(self):
        """Stop NFZ monitoring."""
        self.monitoring = False
        if self.master:
            self.master.close()
        
        self.log_queue.put(None)
        self.log_thread.join()
        self.log("NFZ monitoring stopped")

def main():
    monitor = NFZMonitor(
        log_path = "/home/ubuntu/src/geoFence/log/",
        geo_path = "/home/ubuntu/src/geoFence/geoData/",
        connection_string = "udp:127.0.0.1:14550"
    )
    monitor.start_monitoring()

if __name__ == "__main__":
    main()
