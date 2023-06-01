# ros
import rclpy as ros
from rclpy.node import Node

# ros_msg
from std_msgs.msg import Bool as boolMsg
from std_msgs.msg import String as strMsg
from std_msgs.msg import Float32 as floatMsg

# others
import numpy as np
import json
from pathlib import Path
from datetime import datetime
import time


class PIDNode(Node):
    
    CONFIG_FILE = "pid.cfg"
    
    TOPIC = {
        "LOG": "/log/nodes/pid",
        "DEBUG": "/config/nodes/pid/enable_logging",
        "P": "/config/nodes/pid/proportional",
        "I": "/config/nodes/pid/integral",
        "D": "/config/nodes/pid/differntial",
        "SPEED_IN": "/pid/speed",
        "SPEED_OUT": "/speed/speed",
        "STEER_IN": "/pid/steering",
        "STEER_OUT": "/steering/steering"
        }
    
    def __init__(self):
        super().__init__("PID")
        if not Path(self.CONFIG_FILE).exists():
            with open(self.CONFIG_FILE, "w+") as cfg_file:
                json.dump({}, cfg_file)
        self.create_sub_pub()
        self.load_config()
        self.log("initilized")
        self.c_steer = 0.0
        self.i_steer = 0.0
        self.d_steer = 0.0
        self.ts = self.timestamp()
        self.last_adjustment = 0.0
    
    def timestamp(self):
        return int(round(time.time())/1000)

    def save_config(self):
        json_data = {
            "p": self.p,
            "i": self.i,
            "d": self.d,
            "debug"    : self.enable_debug
        }
        with open(self.CONFIG_FILE, "w") as cfg_file:
            json.dump(json_data, cfg_file)
        self.log(f"saved config to {self.CONFIG_FILE}: {json_data}")
    
    def load_config(self):
        with open(self.CONFIG_FILE, "r") as cfg_file:
            json_data = json.load(cfg_file)
            self.p = json_data["p"] if "p" in json_data else 1
            self.i = json_data["i"] if "i" in json_data else 0
            self.d = json_data["d"] if "d" in json_data else 0
            self.enable_debug = not "debug" in json_data or json_data["debug"]
            self.log(f"loaded config from {self.CONFIG_FILE}: {json_data}")
             
    def create_sub_pub(self):
        self.sub = [
            self.create_subscription(floatMsg, self.TOPIC["P"], self.set_p, 10),
            self.create_subscription(floatMsg, self.TOPIC["I"], self.set_i, 10),
            self.create_subscription(floatMsg, self.TOPIC["D"], self.set_d, 10),
            self.create_subscription(floatMsg, self.TOPIC["SPEED_IN"], self.speed, 10),
            self.create_subscription(floatMsg, self.TOPIC["STEER_IN"], self.steer, 10),
            self.create_subscription(boolMsg, self.TOPIC["DEBUG"], self.enable_logging, 10)
        ]
        self.pub = {
            "SPEED_OUT":   self.create_publisher(floatMsg, self.TOPIC["SPEED_OUT"], 10),
            "STEER_OUT": self.create_publisher(floatMsg, self.TOPIC["STEER_OUT"], 10),
            "LOG": self.create_publisher(strMsg, self.TOPIC["LOG"], 10)
        }
    
    def enable_logging(self, msg:boolMsg):
        self.enable_debug = msg.data
        self.save_config()
        self.log(f"set debug to {self.enable_debug}")

    def log(self, log_text):
        if self.enable_debug:
            msg = strMsg()
            msg.data = f"{datetime.now()} [PIDNode]: {log_text}"
            self.pub["LOG"].publish(msg)
            print(msg.data)

    def set_i(self, msg:floatMsg):
        self.i = msg.data
        self.save_config()
    
    def set_p(self, msg:floatMsg):
        self.p = msg.data
        self.save_config()

    def set_d(self, msg:floatMsg):
        self.d = msg.data
        self.save_config()
    
    def speed(self, msg:floatMsg):
        self.pub["SPEED_OUT"].publish(msg)

    def steer(self, msg:floatMsg):
        adjustment = msg.data - self.c_steer
        newTs = self.timestamp()
        dt = newTs - self.ts
        self.i_steer += dt*adjustment
        self.d_steer = ((adjustment - self.last_adjustment)/dt if dt < 0 else 0)
        self.ts = newTs
        self.c_steer += adjustment * self.p + self.i * self.i_steer + self.d * self.d_steer
        self.c_steer = max(-0.8, min(0.8, self.c_steer))
        outMsg = floatMsg()
        outMsg.data = self.c_steer
        self.pub["STEER_OUT"].publish(outMsg)
        self.log(f"recived {msg.data} steering to {self.c_steer}")
        self.last_adjustment = adjustment

def main(args=None):
    ros.init()
    ros.spin(PIDNode())
    ros.shutdown()

if __name__ == '__main__':
    main()
