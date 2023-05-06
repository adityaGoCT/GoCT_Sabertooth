#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pysabertooth import Sabertooth
import serial.tools.list_ports as port
from std_msgs.msg import Float32MultiArray


class SabertoothControl(Node):

    def __init__(self, saber_baudrate, saber_addr, saber_timeout):
        super().__init__("sabertooth_driver")
        self.BAUDRATE = saber_baudrate
        self.SABERTOOTH_ADDRESS = saber_addr
        self.SABERTOOTH_TIMEOUT = saber_timeout
        self.motor_cmd_sub = self.create_subscription(
            Float32MultiArray, "motor_cmd", self.motor_cmd_cb, 1)
        self.vel = Float32MultiArray()

    def initialize_sabertooth(self):
            self.get_logger().info("Detecting sabertooth at USB0")
            self.saber = Sabertooth(
                "/dev/ttyUSB0",
                self.BAUDRATE,
                self.SABERTOOTH_ADDRESS,
            )


    def motor_cmd_cb(self, data):
        self.v_l = data.data[0]
        self.v_r = data.data[1]
        self.saber.driveBoth( self.v_l,self.v_r)

    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    sc = SabertoothControl(saber_baudrate=9600,
                           saber_addr=128, saber_timeout=0.1)

    sc.initialize_sabertooth()
    rclpy.spin(sc)
    sc.main()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
