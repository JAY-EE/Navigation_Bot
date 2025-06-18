
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init(args=None)
node = rclpy.create_node('GPS_Publisher')
qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE)
gps_pub = node.create_publisher(String,'gps_publisher',qos_profile)

def read_gps(port='/dev/ttyUSB1', baudrate=9600):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Listening for GPS data on {port}...")
        
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):  # Standard GPS position fix
                print(f"GPS Data: {line}")
                msg = String()
                msg.data = line  # Assign the string data to the ROS 2 message
                gps_pub.publish(msg)  # Publish the message


    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopping GPS reader.")

if __name__ == "__main__":
    read_gps()



# #!/usr/bin/env python3
# import serial

# def parse_gga(gga_string):
#     # Split the string by commas
#     fields = gga_string.split(',')
    
#     if len(fields) < 15:  # GGA should have 15 fields
#         return None
        
#     # Create a dictionary with named fields
#     gga_data = {
#         'message_id': fields[0],        # $GNGGA
#         'utc_time': fields[1],          # HHMMSS.sss
#         'latitude': fields[2],          # DDMM.MMMMM
#         'lat_direction': fields[3],     # N or S
#         'longitude': fields[4],         # DDDMM.MMMMM
#         'lon_direction': fields[5],     # E or W
#         'fix_quality': fields[6],       # 0=no fix, 1=GPS fix, 2=DGPS fix
#         'num_satellites': fields[7],    # Number of satellites in use
#         'hdop': fields[8],             # Horizontal dilution of precision
#         'altitude': fields[9],          # Antenna altitude above mean sea level
#         'altitude_units': fields[10],   # Meters (always M)
#         'geoid_height': fields[11],     # Height of geoid above WGS84 ellipsoid
#         'geoid_height_units': fields[12], # Meters (always M)
#         'time_since_dgps': fields[13],  # Time since last DGPS update
#         'dgps_station_id': fields[14].split('*')[0]  # DGPS station ID
#     }
    
#     # Convert and format time
#     if gga_data['utc_time']:
#         try:
#             time = gga_data['utc_time'].split('.')[0]
#             hours = time[0:2]
#             minutes = time[2:4]
#             seconds = time[4:6]
#             gga_data['formatted_time'] = f"{hours}:{minutes}:{seconds}"
#         except IndexError:
#             gga_data['formatted_time'] = None

#     return gga_data

# def read_gps(port='/dev/ttyUSB1', baudrate=9600):
#     try:
#         ser = serial.Serial(port, baudrate, timeout=1)
#         print(f"Listening for GPS data on {port}...")
        
#         while True:
#             line = ser.readline().decode('utf-8', errors='ignore').strip()
#             if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
#                 parsed_data = parse_gga(line)
#                 if parsed_data:
#                     print("\nParsed GPS Data:")
#                     print(f"Time (UTC): {parsed_data['formatted_time']}")
#                     print(f"Fix Quality: {parsed_data['fix_quality']}")
#                     print(f"Number of Satellites: {parsed_data['num_satellites']}")
#                     print(f"HDOP: {parsed_data['hdop']}")
                    
#                     # Print location data if available
#                     if parsed_data['latitude'] and parsed_data['longitude']:
#                         print(f"Latitude: {parsed_data['latitude']} {parsed_data['lat_direction']}")
#                         print(f"Longitude: {parsed_data['longitude']} {parsed_data['lon_direction']}")
#                         print(f"Altitude: {parsed_data['altitude']} {parsed_data['altitude_units']}")
#                     else:
#                         print("Location: No Fix")

#     except serial.SerialException as e:
#         print(f"Serial error: {e}")
#     except KeyboardInterrupt:
#         print("\nStopping GPS reader.")

# if __name__ == "__main__":
#     read_gps()