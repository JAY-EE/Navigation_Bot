
# MASTER SERVER (RUNS ON ROVER)

""" 
Setup the communication from rover to base

# Recive :

1) Arm and Rover commands from base sataion (Websocket) (master_client.py file)
   Rover ---> rover_port

2) GPS and Angles feedback from (arm_server.py and rover_server.py)
   Feedback Angles ---> encoder_angles
   GPS ---> gps_data

# Publish :

1) Arm and Rover commands to pyserial python files
   Arm ---> arm_server_controller
   Rover ---> cmd_vel

"""

import asyncio
import websockets
import json
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from threading import Lock
from datetime import datetime, timedelta
from sensor_msgs.msg import Imu


Ip = 'localhost'    # Permannet IP of rover 

# Ip = "10.74.3.125"

# Channel ... Should be the same on master server and client !     
port_chassis = 8766
port_imu = 8767
port_gps = 8768

chassis_server = None
gps_server = None
imu_server = None

COMMAND_TIMEOUT = 1  # failsafe timing 1 sec


class WebSocketServer(Node):
    def __init__(self):
        super().__init__('Master_Server_Node')

        # Locks for thread-safe data access
        self._gps_lock = Lock()
        self._chassis_lock = Lock()
        self._imu_lock = Lock()

        # Initialize data varibales
        self._gps_data = None
        self._imu_data = None

        # For fail safe timing
        self._last_chassis_command_time = datetime.now()

        # Publishers and subscribers
        self.pub_chassis = self.create_publisher(Twist, '/cmd_vel', 10)

        self.gps_sub = self.create_subscription(
            String,
            'gps_publisher',
            self.gps_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu_publisher',
            self.imu_callback,
            10)
    
        # Event looping through threding
        self.loop = asyncio.new_event_loop()
        self.websocket_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self.websocket_thread.start()

    
    def gps_callback(self,msg):
        with self._gps_lock:
            self._gps_data = msg.data
            # print(self._gps_data)

    async def gps_handle(self,websocket):
        while True:
            try:
                with self._gps_lock:
                    gps_data = self._gps_data
                
                if gps_data is not None:
                    await websocket.send(gps_data)
                    self.get_logger().info(f"GPS Sent : {gps_data}")
                await asyncio.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Error in gps_handle: {e}")
                await asyncio.sleep(1)

    def imu_callback(self, msg):
        with self._imu_lock:
            # Convert IMU message to a serializable dictionary
            imu_dict = {
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                }
            }
            self._imu_data = json.dumps(imu_dict)

    async def imu_handle(self, websocket):
        while True:
            try:
                with self._imu_lock:
                    imu_data = self._imu_data
                
                if imu_data is not None:
                    await websocket.send(imu_data)
                    self.get_logger().info(f"IMU data sent : {imu_data}")
                await asyncio.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Error in imu_handle: {e}")
                await asyncio.sleep(1)


    async def handle_chassis(self, websocket):      # Rover command from base
        try:
            async for message in websocket:
                self.get_logger().info(f"Received message for rover: {message}")
                data = json.loads(message)
                chassis_command = Twist()
                chassis_command.linear.x = data.get("linear", {}).get("x", 0.0)
                chassis_command.linear.y = data.get("linear", {}).get("y", 0.0)
                chassis_command.linear.z = data.get("linear", {}).get("z", 0.0)
                chassis_command.angular.z = data.get("angular", {}).get("z", 0.0)
                self.pub_chassis.publish(chassis_command)
                with self._chassis_lock:
                    self._last_chassis_command_time = datetime.now()  #failsafe 
        except Exception as e:
            self.get_logger().error(f"Error in handle_chassis: {e}")

    async def monitor_timeouts(self):            # failsafe mechanism 
        while True:
            now = datetime.now()
            # Handle chassis timeout
            with self._chassis_lock:
                if (now - self._last_chassis_command_time).total_seconds() > COMMAND_TIMEOUT:
                    # self.get_logger().warning("No chassis command received. Sending 0 values.")
                    chassis_command = Twist()
                    self.pub_chassis.publish(chassis_command)
            await asyncio.sleep(0.1)



    async def start_server(self):     # Intialise the server !!!
        chassis_server = await websockets.serve(self.handle_chassis, Ip, port_chassis, ping_timeout=None)
        gps_server = await websockets.serve(self.gps_handle, Ip, port_gps, ping_timeout=None)
        imu_server = await websockets.serve(self.imu_handle, Ip, port_imu, ping_timeout=None)
        self.get_logger().info(f"WebSocket server started on ws://{Ip}")

        await asyncio.gather(
            chassis_server.wait_closed(),
            gps_server.wait_closed(),
            imu_server.wait_closed(),
        )

    def _run_websocket_server(self):          # Run the server in a separate thread
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(asyncio.gather(
            self.start_server(),
            self.monitor_timeouts()
        ))
        self.loop.run_forever()



    def run(self):            # execute the threads of ROS2!!!
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass



def main(args=None):         # Run all the functions.
    rclpy.init(args=args)
    server = WebSocketServer()
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
