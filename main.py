import argparse, os, time, json, math, threading, queue
from pymavlink import mavutil
from datetime import datetime
from geo_utils import GeoFence, Logger

class Mav:
    def __init__(self, log_path = "/home/ubuntu/src/geoFence/log", connection_string = "udp:127.0.0.1:14550"):
        self.logger = Logger(log_path)

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None  # mm to meters

        self.connection_string = connection_string

        self.rtl_triggered = False
        self.current_flight_mode = None
        self.original_flight_mode = None

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

    def connect_mavlink(self, connection_string = "udp:127.0.0.1:14550"):
        """Establish MAVLink connection."""
        try:
            self.logger.write(f"Connecting to MAVLink: {self.connection_string}")
            
            # Handle serial device connections with explicit baud rate
            if self.connection_string.startswith('/dev/') or self.connection_string.startswith('COM'):
                # For serial devices, ensure baud rate is specified
                if ':' not in self.connection_string:
                    self.connection_string += ':57600'  # Default baud rate
                    self.logger.write(f"Added default baud rate: {self.connection_string}")
                
                # Parse serial device and baud rate
                parts = self.connection_string.split(':')
                device = parts[0]
                baud = int(parts[1]) if len(parts) > 1 else 57600
                self.logger.write(f"Connecting to serial device: {device} at {baud} baud")
                self.master = mavutil.mavlink_connection(device, baud=baud)
            else:
                # UDP or other connection types
                self.master = mavutil.mavlink_connection(self.connection_string)
            
            # Wait for heartbeat with timeout
            self.logger.write("Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)
            self.logger.write(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}")
            
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
            self.logger.write(f"Failed to connect to MAVLink: {e}")
            return False

    def heartbeat_handle(self):
        try:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg is not None:
                custom_mode = msg.custom_mode
                mode_name = self.get_flight_mode_name(custom_mode)

                if self.current_flight_mode != custom_mode:
                    self.logger.write(f"Flight mode changed: {mode_name} (mode {custom_mode})")
                    self.current_flight_mode = custom_mode

                if not self.rtl_triggered and custom_mode != 6:  # 6 = RTL
                    self.original_flight_mode = custom_mode
                
                return custom_mode
            return None

        except Exception as e:
            self.logger.write(f"Error updating flight mode: {e}")  
            return None  

    def gps_handle(self):
        try:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg is not None:
                self.current_lat = msg.lat / 1e7
                self.current_lon = msg.lon / 1e7
                self.current_alt = msg.relative_alt / 1000.0  # mm to meters
                self.logger.write(f"Status: Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}, Alt={self.current_alt:.1f}m")
                # self.load_geojson()

                return self.current_lat, self.current_lon, self.current_alt
            return None, None, None

        except Exception as e:
            self.logger.write(f"Error processing GPS data: {e}")
            return None, None, None

    def send_warning(self, distance: float, nfz_name: str):
        """Send warning message (could be extended to send to GCS)."""
        self.logger.write(f"WARNING: Approaching NFZ '{nfz_name}' - Distance: {distance:.1f}m")
        
        # Could send MAVLink text message to Ground Control Station
        try:
            warning_msg = f"WARNING: NFZ {distance:.0f}m"
            self.master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_WARNING,
                warning_msg.encode()[:50]  # Max 50 chars
            )
        except Exception as e:
            self.logger.write(f"Failed to send warning message: {e}")

    def send_break_command(self):
        """Send BREAK command to drone - RTL (Return To Launch) mode."""
        try:
            # Store current mode as original if not already stored
            if self.original_flight_mode is None and self.current_flight_mode is not None:
                self.original_flight_mode = self.current_flight_mode
                mode_name = self.get_flight_mode_name(self.current_flight_mode)
                self.logger.write(f"Storing original flight mode: {mode_name} (mode {self.current_flight_mode})")
            
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
            self.logger.write("BREAK COMMAND SENT - Drone commanded to RTL (Return To Launch)")
            return True
            
        except Exception as e:
            self.logger.write(f"Failed to send RTL command: {e}")
            return False

    def restore_original_mode(self):
        """Restore the original flight mode when safe distance is reached."""
        if self.original_flight_mode is None:
            self.logger.write("No original flight mode to restore")
            return False
            
        try:
            mode_name = self.get_flight_mode_name(self.original_flight_mode)
            self.logger.write(f"Restoring original flight mode: {mode_name} (mode {self.original_flight_mode})")
            
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
            self.logger.write(f"Failed to restore flight mode: {e}")
            return False
    
