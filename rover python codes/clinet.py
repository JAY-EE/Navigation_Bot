# CLIENT (RUNS ON BASE)

""" 
Setup the communication from Joystick

# Publish :
1) Rover jostick commands
   Rover ---> rover_client

"""

import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys


# Initialize Pygame and ROS 2
pygame.init()
pygame.joystick.init()
rclpy.init(args=None)
node = rclpy.create_node('Client_Server')

# Global variables 

joysticks = []

kfront = 100
kleftright = 175
frontback = 0
rightleft = 0
msg_rover = 0
x = 0
joystick_rover = None

# Initialize publishers
publisher_rover = node.create_publisher(Twist, '/rover_client', 10)

def setup_joystick():
    global joystick_arm, joystick_rover

    joystick_count = pygame.joystick.get_count()
    print(f"Joystick count: {joystick_count}")

    if joystick_count == 1:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick detected: {joystick.get_name()}")
        joystick_rover = joystick

    # elif joystick_count > 1:
    #     joysticks = [pygame.joystick.Joystick(i) for i in range(2)]
    #     joysticks[0].init()
    #     joysticks[1].init()

    #     print(f"Joystick 1 detected: {joysticks[0].get_name()}")
    #     print(f"Joystick 2 detected: {joysticks[1].get_name()}")

    #     if joysticks[0].get_name() == "Sony Interactive Entertainment Wireless Controller" or "Wireless Controller":
    #         joystick_rover = joysticks[0]
    #         joystick_arm = joysticks[1]
    #     else:
    #         joystick_arm = joysticks[0]
    #         joystick_rover = joysticks[1]

    else:
        print("No Joysticks found.")
        pygame.quit()
        sys.exit()


def process_events():

    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            joystick = pygame.joystick.Joystick(event.device_index)
            joystick.init()
            print(f"Joystick {joystick.get_name()} added.")
            setup_joystick()


def update_joystick():  # set the joystick buttons ..
    
    global kfront, kleftright, msg_rover, x

    msg_rover = Twist()

    if joystick_rover:
        frontback = joystick_rover.get_axis(1)
        rightleft = joystick_rover.get_axis(0)

        if joystick_rover.get_button(4) == 1:    # increase the pwm of rover 
            kfront += 2 if kfront<254 else 0
            kleftright += 1 if kfront<254 else 0

        if joystick_rover.get_button(5) == 1 and kfront > 1:
            kfront -= 2
            kleftright -= 1 if kleftright > 1 else 0

        if abs(frontback) > 0.2:
            msg_rover.linear.x = float(-frontback * kfront)
        else:
            msg_rover.linear.x = 0.0

        if abs(rightleft) > 0.2:
            msg_rover.angular.z = float(-rightleft * kleftright)
        else:
            msg_rover.angular.z = 0.0

        rover_msg = {
            "linear": msg_rover.linear.x,
            "angular": msg_rover.angular.z
        }

        print(rover_msg)


def publish_data():
    global msg_rover

    publisher_rover.publish(msg_rover)

def main_loop():

    while rclpy.ok():
        pygame.event.pump()

        update_joystick()

        publish_data()

        process_events()

        rclpy.spin_once(node, timeout_sec= 0.01)

        # Exit condition
        if joysticks and joysticks[0].get_button(10):
            print("Tata Bye Bye ... See You Soon Again !!!")
            break

    # Shutdown once loop exits
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


def main():
    global joystick_rover
    setup_joystick()
    main_loop()

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()

