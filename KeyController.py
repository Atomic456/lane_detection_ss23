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
        "SPEED_MAX": "/config/nodes/key_controller/speed_max",
        "SPEED_MIN": "/config/nodes/key_controller/speed_min",
        "SPEED_INC": "/config/nodes/key_controller/speed_inc",
        "SPEED_DEC": "/config/nodes/key_controller/speed_dec",
        "STEER_MAX": "/config/nodes/key_controller/steer_max",
        "STEER_MIN": "/config/nodes/key_controller/steer_min",
        "STEER_INC": "/config/nodes/key_controller/steer_inc",
        "STEER_DEC": "/config/nodes/key_controller/steer_dec",
        "LOG": "/log/nodes/key_controller"
    }

    def __init__(self):
        super().__init__("KeyControl")
        if not Path(self.CONFIG_FILE).exists():
            with open(self.CONFIG_FILE, "w+") as cfg_file:
                json.dump({}, cfg_file)
        self.bridge = CvBridge()
        self.sub = {
            "IMG": self.create_subscription(imgMsg, self.TOPIC["IMG"], self.show, 10),
            "SPEED_MAX": self.create_subscription(floatMsg, self.TOPIC["SPEED_MAX"], self.set_speed_max, 10),
            "SPEED_MIN": self.create_subscription(floatMsg, self.TOPIC["SPEED_MIN"], self.set_speed_min, 10),
            "SPEED_INC": self.create_subscription(floatMsg, self.TOPIC["SPEED_INC"], self.set_speed_inc, 10),
            "SPEED_DEC": self.create_subscription(floatMsg, self.TOPIC["SPEED_DEC"], self.set_speed_dec, 10),
            "STEER_MAX": self.create_subscription(floatMsg, self.TOPIC["STEER_MAX"], self.set_steer_max, 10),
            "STEER_MIN": self.create_subscription(floatMsg, self.TOPIC["STEER_MIN"], self.set_steer_min, 10),
            "STEER_INC": self.create_subscription(floatMsg, self.TOPIC["STEER_INC"], self.set_steer_inc, 10),
            "STEER_DEC": self.create_subscription(floatMsg, self.TOPIC["STEER_DEC"], self.set_steer_dec, 10)
        }
        self.pub = {
            "SPEED": self.create_publisher(floatMsg, self.TOPIC["SPEED"], 10),
            "STEER": self.create_publisher(floatMsg, self.TOPIC["STEER"], 10),
            "LOG":   self.create_publisher(strMsg, self.TOPIC["LOG"], 10)
        }
        self.load_config()
        self.stop()
        self.speed_last = 0
        self.steering_last = 0

    def enable_logging(self, msg:boolMsg):
        self.enable_debug = msg.data
        self.save_config()

    def set_speed_max(self, msg:floatMsg):
        self.speedMax = msg.data
        self.save_config()

    def set_speed_min(self, msg:floatMsg):
        self.speedMin = msg.data
        self.save_config()

    def set_speed_inc(self, msg:floatMsg):
        self.speedInc = msg.data
        self.save_config()

    def set_speed_dec(self, msg:floatMsg):
        self.speedDec = msg.data
        self.save_config()

    def set_steer_max(self, msg:floatMsg):
        self.steerMax = msg.data
        self.save_config()

    def set_steer_min(self, msg:floatMsg):
        self.steerMin = msg.data
        self.save_config()

    def set_steer_inc(self, msg:floatMsg):
        self.steerInc = msg.data
        self.save_config()

    def set_steer_dec(self, msg:floatMsg):
        self.steerDec = msg.data
        self.save_config()

    def save_config(self):
        json_data = {
            "debug": self.enable_debug,
            "keyControl": self.enable_key_control,
            "disable": self.emergency_disable,
            "speedMax": self.speedMax,
            "speedMin": self.speedMin,
            "speedInc": self.speedInc,
            "speedDec": self.speedDec,
            "steerMax": self.steerMax,
            "steerMin": self.steerMin,
            "steerInc": self.steerInc,
            "steerDec": self.steerDec
        }
        with open(self.CONFIG_FILE, "w") as cfg_file:
            json.dump(json_data, cfg_file)
    
    def load_config(self):
        with open(self.CONFIG_FILE, "r") as cfg_file:
            json_data = json.load(cfg_file)
            self.enable_debug = not "debug" in json_data or json_data["debug"]
            self.enable_key_control = not "keyControll" in json_data or json_data["keyControll"]
            self.emergency_disable = "disable" in json_data and json_data["disable"]
            self.steerMax = json_data["steerMax"] if "steerMax" in json_data else 0.8
            self.steerMin = json_data["steerMin"] if "steerMin" in json_data else -0.8
            self.steerInc = json_data["steerInc"] if "steerInc" in json_data else 0.1
            self.steerDec = json_data["steerDec"] if "steerDec" in json_data else 0.1
            self.speedMax = json_data["speedMax"] if "speedMax" in json_data else 1.0
            self.speedMin = json_data["speedMin"] if "speedMin" in json_data else -1.0
            self.speedInc = json_data["speedInc"] if "speedInc" in json_data else 0.33
            self.speedDec = json_data["speedDec"] if "speedDec" in json_data else 0.33

    def log(self, log_text):
        if self.enable_debug:
            msg = strMsg()
            msg.data = f"{datetime.now()} [SaveNode]: {log_text}"
            self.pub["LOG"].publish(msg)
            print(msg.data)

    def stop(self):
        self.speed = 0.0
        self.steering = 0.0
        self.log("reset")
        self.send_speed()
        self.send_steer()

    def send_steer(self):
        msg = floatMsg()
        msg.data = self.steering
        self.pub["STEER"].publish(msg)
        self.log(f"steering value={self.steering}")

    def send_speed(self):
        msg = floatMsg()
        msg.data = self.speed
        self.pub["SPEED"].publish(msg)
        self.log(f"speed value={self.speed}")

    def show(self, msg:imgMsg):
        image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.rotate(image, cv2.ROTATE_180)
        cv2.imshow("Bild-Autofahren", image)
        key = cv2.pollKey()

        self.speed_last = self.speed
        self.steering_last = self.steering

        if (key != -1):
            match (chr(key)):
                case 'w'|'W': 
                    self.speed += self.speedInc if self.enable_key_control and not self.emergency_disable else 0
                case 's'|'S':
                    self.speed -= self.speedDec if self.enable_key_control and not self.emergency_disable else 0
                case 'a'|'A':
                    self.steering -= self.steerDec if self.enable_key_control and not self.emergency_disable else 0
                case 'd'|'D':
                    self.steering += self.steerInc if self.enable_key_control and not self.emergency_disable else 0
                case 'r'|'R':
                    self.stop()
                case 'q'|'Q':
                    self.steering = 0.0
                case 'x'|'X':
                    self.enable_key_control = not self.enable_key_control
                    self.log("en" if self.enable_key_control else "dis" + "abled wasd controll")
                case ' ':
                    self.emergency_disable = not self.emergency_disable
                    self.stop()
                    self.log("en" if self.emergency_disable else "dis" + "abled emergency stop")

            self.speed = self.speedMax if self.speed > self.speedMax else self.speed if self.speed > self.speedMin else self.speedMin
            self.steering = self.steerMax if self.steering > self.steerMax else self.steering if self.steering > self.steerMin else self.steerMin

        if self.enable_key_control and not self.emergency_disable:
            if self.speed != self.speed_last:
                self.send_speed()
            if self.steering != self.steering_last:
                self.send_steer()

def main(args=None):
    ros.init()
    ros.spin(KeyController())
    ros.shutdown()

if __name__ == "__main__":
    main()
