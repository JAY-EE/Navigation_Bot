
# MASTER CLIENT (RUNS ON BASE)

""" 
Setup the communication from base to rover

# Recive :

1) Arm and Rover commands from joystick (client.py file)
   Arm ---> arm_client
   Rover ---> rover_client

2) GPS and Angles feddback from rover

# Publish :

1) GPS and Angles feedback for GUI and other files
   GPS ---> gps_publisher
   Feedback ---> joint_angles_json

"""

# Things to remeber :
# IP should match with IP of server


import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

Ip = "localhost"

# Ip = "10.61.44.221"     # Rover IP (Should match from master_server)

# Channels should match server channel ..
port_chassis = 8766
port_imu = 8767
port_gps = 8768


# Global Variables
websocket_rover = None
websocket_gps = None
websocket_imu = None


class WebSocketClient(Node):
    def __init__(self):
        super().__init__('Master_Client_Node')

        # Create Subscription 
        self.create_subscription(Twist, '/rover_client', self.callback_chassis, 10)

        # Create Publishers 
        self.gps_pub = self.create_publisher(String,'gps',10)
        self.imu_pub = self.create_publisher(String,'imu',10)
    
        # Start the WebSocket Connection
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.setup_connections())


    async def setup_connections(self):           # Setup Communication
        global  websocket_rover, websocket_gps, websocket_imu

        try:
            websocket_rover = await websockets.connect(f'ws://{Ip}:{port_chassis}', ping_interval=20, ping_timeout=10)
            websocket_gps = await websockets.connect(f'ws://{Ip}:{port_gps}', ping_interval=20, ping_timeout=10)
            websocket_imu = await websockets.connect(f'ws://{Ip}:{port_imu}', ping_interval=20, ping_timeout=10)
            asyncio.create_task(self.receive_gps_data())
            asyncio.create_task(self.receive_imu_data())
            self.get_logger().info(f"Connected to WebSocket server on {Ip}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to WebSocket server: {e}")
            await asyncio.sleep(5)                # Retry Connection
            await self.setup_connections() 


    def callback_chassis(self, msg):              # Recieve Rover data from Rover josytick
        x = msg.linear.x
        y = msg.linear.y
        p = msg.linear.z
        ang_z = msg.angular.z

        message = {
            "linear": {"x": x, "y": y, "z": p},
            "angular": {"z": ang_z}
        }
        asyncio.create_task(self.send_rover_server(message))
  

    async def send_rover_server(self, msg):       # Sends the Rover commands to rover.
        global websocket_rover

        if websocket_rover:
            try:
                await websocket_rover.send(json.dumps(msg))
                self.get_logger().info(f"Sent to Rover : {msg}")
            except websockets.exceptions.ConnectionClosedError as e:
                self.get_logger().error(f"Connection to rover. Reconnecting ... : {e}")
                await self.setup_connections()
                await self.send_rover_server(msg)
            except Exception as e:
                self.get_logger().error(f"Error : {e}")
        else:
            self.get_logger().warn("WebSocket connection to rover server is not established yet.")

    async def receive_gps_data(self):
        global websocket_gps
        try:
            while websocket_gps:  # Add a check to make sure websocket_gps is not None
                try:
                    message = await websocket_gps.recv()  # Use recv() instead of async for
                    fields = message.split(',')
                    if len(fields) < 15:  # GGA should have 15 fields
                        continue
                    gga_data = {
                        'latitude': fields[2],          # DDMM.MMMMM
                        'lat_direction': fields[3],     # N or S
                        'longitude': fields[4],         # DDDMM.MMMMM
                        'lon_direction': fields[5],     # E or W
                        'altitude': fields[9],          # Antenna altitude above mean sea level
                        'altitude_units': fields[10],   # Meters (always M)
                        'num_satellites': fields[7],    # Number of satellites in use
                        'hdop': fields[8],             # Horizontal dilution of precision
                    }

                    gps_msg = String()
                    gps_msg.data = json.dumps(gga_data)
                    self.gps_pub.publish(gps_msg)
                    # self.get_logger().info(f"Received GPS data: {gga_data}")
                    
                except websockets.exceptions.ConnectionClosed:
                    self.get_logger().warn("GPS connection closed. Reconnecting...")
                    break 
        except Exception as e:
            self.get_logger().error(f"Error receiving GPS data: {e}")

    async def receive_imu_data(self):
        global websocket_imu
        try:
            while websocket_imu:
                try:
                    message = await websocket_imu.recv()
                    imu_msg = String()
                    imu_msg.data = message 
                    self.imu_pub.publish(imu_msg)
                    # self.get_logger().info("Received IMU data :")
                    
                except websockets.exceptions.ConnectionClosed:
                    self.get_logger().warn("IMU connection closed. Reconnecting...")
                    break
                except Exception as e:
                    self.get_logger().error(f"Error processing IMU data: {e}")
                    await asyncio.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Error in receive_imu_data: {e}")
    

async def main_async():          # Intialise the base server and communication.
    rclpy.init(args=None)
    node = WebSocketClient()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()