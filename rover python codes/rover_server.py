# ROVER_SERVER (RUNS ON ROVER)

""" 
Setup the communication from Rover to ROVER ESP

# Recive :

1) Rover commands master_server
   Arm ---> arm_controller_server

# Publish :

1) GPS to the master_server
   GPS ---> gps_data

"""

""""
Things to remember :
i) It automatically finds the desired port and connects no need to chnage.
ii) Wait for few seconds .. may not connect in once but will !

"""


import serial
import serial.tools.list_ports
import time
import json
import rclpy
from rclpy.node import Node
import time
import serial
import json
from geometry_msgs.msg import Twist


def find_serial_port():         # Scan available serial ports and 
                                # return the one that responds with 'rover'

    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            print(port.device)
            try:
                ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
                time.sleep(2)  # Some time for the serial connection to stabilize
                
                # Send identification command
                identification_command = json.dumps({"command": "identify"}) + '\n'
                ser.write(identification_command.encode())

                # check wheather data is corrupted or not 
                try :
                    response = ser.readline().decode().strip()
                except UnicodeDecodeError as e:
                        print(f"UTF-8 decode error: {e}")
                        continue
                
                # Read response
                if response:
                    data = json.loads(response)
                    if data.get("device_type") == "rover":
                        print(f"Connected to {port.device}")
                        return ser
                ser.close()
            except (serial.SerialException, json.JSONDecodeError):
                pass  # Try the next port
    return None


linear = 0
angular = 0 

def rover_callback(data):         # Read value from /cmd_vel
    global linear, angular

    linear = float(data.linear.x)
    angular = float(data.angular.z)



# ROS2 node setup !

rclpy.init()
node = Node('Rover_Server')
node.create_subscription(Twist, '/cmd_vel', rover_callback, 10)


def main():
    global node, linear, angular

    serial_connection = None

    while rclpy.ok:                    # Check if ROS2 commands are available !
        rclpy.spin_once(node)

        if serial_connection is None:
            print("Searching for the rover...")
            serial_connection = find_serial_port()
            if serial_connection:
                print("Rover connected!")
            else:
                print("Rover not found. Retrying...")
                time.sleep(2)
                continue

        try:

            response = {
                    "linear": linear,
                    "angular": angular
                }
            
            # send repsonse to master-server
            serial_connection.write((json.dumps(response) + '\n').encode())


        except serial.SerialException:
            print("Connection lost. Reconnecting...")
            serial_connection.close()
            serial_connection = None

        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
