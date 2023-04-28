# opencv
import cv2
from cv_bridge import CvBridge

# ros
import rclpy as ros
from rclpy.node import Node

# ros_msg
from std_msgs.msg import Float32 as floatMsg
from std_msgs.msg import String as strMsg
from std_msgs.msg import Bool as boolMsg
from sensor_msgs.msg import Image as imgMsg

# others
from datetime import datetime
import json
from pathlib import Path

class KeyController(Node):

    CONFIG_FILE = "key_controller.cfg"

    TOPIC = {
        "IMG": "/perception/image_gray8",
        "SPEED": "/speed/speed",
        "STEER": "/steering/steering",
        "LOG": "/log/nodes/key_controller"
    }

    def __init__(self):
        super().__init__("KeyControl")
        if not Path(self.CONFIG_FILE).exists():
            with open(self.CONFIG_FILE, "w+") as cfg_file:
                json.dump({}, cfg_file)
        self.bridge = CvBridge()
        self.sub = {
            "IMG": self.create_subscription(imgMsg, self.TOPIC["IMG"], self.show, 10)
        }
        self.pub = {
            "SPEED": self.create_publisher(floatMsg, self.TOPIC["SPEED"], 10),
            "STEER": self.create_publisher(floatMsg, self.TOPIC["STEER"], 10),
            "LOG":   self.create_publisher(strMsg, self.TOPIC["LOG"], 10)
        }
        self.load_config()
        self.stop()

    def enable_logging(self, msg:boolMsg):
        self.enable_debug = msg.data
        self.save_config()

    def save_config(self):
        json_data = {
            "debug": self.enable_debug,
            "keyControl": self.wasdControl,
            "disable": self.emergencyDisable
        }
        with open(self.CONFIG_FILE, "w") as cfg_file:
            json.dump(json_data, cfg_file)
    
    def load_config(self):
        with open(self.CONFIG_FILE, "r") as cfg_file:
            json_data = json.load(cfg_file)
            self.enable_debug = not "debug" in json_data or json_data["debug"]
            self.wasdControl = not "keyControll" in json_data or json_data["keyControll"]
            self.emergencyDisable = "disable" in json_data and json_data["disable"]

    def log(self, log_text):
        if self.enable_debug:
            msg = strMsg()
            msg.data = f"{datetime.now()} [SaveNode]: {log_text}"
            self.pub["LOG"].publish(msg)
            print(msg.data)

    def stop(self):
        self.wasd_speed = 0.0
        self.wasd_steering = 0.0
        self.log("reset")
        self.speed()
        self.steer()

    def steer(self):
        msg = floatMsg()
        msg.data = self.wasd_steering
        self.pub["STEER"].publish(msg)
        self.log(f"steering value={self.wasd_steering}")

    def speed(self):
        msg = floatMsg()
        msg.data = self.wasd_speed
        self.pub["SPEED"].publish(msg)
        self.log(f"speed value={self.wasd_speed}")

    def show(self, msg:imgMsg):
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("Bild-Autofahren", image)
        while ros.ok():
            key = cv2.pollKey()

            if (key != -1):
                match (chr(key)):
                    case 'w'|'W': 
                        self.wasd_speed += 0.15 if self.wasdControl and not self.emergencyDisable else 0
                    case 's'|'S':
                        self.wasd_speed -= 0.15 if self.wasdControl and not self.emergencyDisable else 0
                    case 'a'|'A':
                        self.wasd_steering -= 0.2 if self.wasdControl and not self.emergencyDisable else 0
                    case 'd'|'D':
                        self.wasd_steering += 0.2 if self.wasdControl and not self.emergencyDisable else 0
                    case 'r'|'R':
                        self.stop()
                    case 'q'|'Q':
                        self.wasd_steering = 0.0
                    case 'x'|'X':
                        self.wasdControl = not self.wasdControl
                        self.log("en" if self.wasdControl else "dis" + "abled wasd controll")
                    case ' ':
                        self.emergencyDisable = not self.emergencyDisable
                        self.stop()
                        self.log("en" if self.emergencyDisable else "dis" + "abled emergency stop")

                self.wasd_speed = 1 if self.wasd_speed > 1 else self.wasd_speed if self.wasd_speed > -1 else -1
                self.wasd_steering = 0.8 if self.wasd_steering > 0.8 else self.wasd_steering if self.wasd_steering > -0.8 else -0.8

            if self.wasdControl and not self.emergencyDisable:
                self.speed()
                self.steer()

def main(args=None):
    ros.init()
    ros.spin(KeyController())
    ros.shutdown()

if __name__ == "__main__":
    main()
