# opencv
import cv2
from cv_bridge import CvBridge

# ros
import rclpy as ros
from rclpy.node import Node

# ros_msg
from std_msgs.msg import Bool as boolMsg
from std_msgs.msg import String as strMsg
from std_msgs.msg import Float32 as floatMsg
from sensor_msgs.msg import Image as imgMsg

# others
from datetime import datetime
import json
from pathlib import Path

class SaveNode(Node):
    
    CONFIG_FILE = "save_node.cfg"
    
    TOPIC = {
        "IMG": "/perception/image_gray8",
        "LOG": "/log/nodes/save_node",
        "SAVE_PATH": "/config/nodes/save_node/save_path",
        "LOAD_PATH": "/config/nodes/save_node/load_path",
        "DO_SAVE": "/config/nodes/save_node/enable_saveing",
        "DO_LOAD": "/config/nodes/save_node/enabe_loading",
        "LOAD_FRQ": "/config/nodes/save_node/load_frequenz",
        "DEBUG": "/config/nodes/save_node/enable_logging"
        }
    
    def __init__(self):
        super().__init__("SaveNode")
        if not Path(self.CONFIG_FILE).exists():
            with open(self.CONFIG_FILE, "w+") as cfg_file:
                json.dump({}, cfg_file)
        self.create_sub_pub()
        self.load_config()
        self.timer = self.create_timer(self.load_frequenz, self.load)
        self.bridge = CvBridge()
        self.save_file_count = 1
        self.load_file_count = 1
        self.log("initilized")
    
    def save_config(self):
        json_data = {
            "load_path": self.load_path,
            "save_path": self.save_path,
            "load_frequenz": self.load_frequenz,
            "do_load"  : self.enable_load,
            "do_save"  : self.enable_load,
            "debug"    : self.enable_debug
        }
        with open(self.CONFIG_FILE, "w") as cfg_file:
            json.dump(json_data, cfg_file)
        self.log(f"saved config to {self.CONFIG_FILE}: {json_data}")
    
    def load_config(self):
        with open(self.CONFIG_FILE, "r") as cfg_file:
            json_data = json.load(cfg_file)
            self.load_path = json_data["load_path"] if "load_path" in json_data else "/home/user/pictures"
            self.load_frequenz = json_data["load_frequenz"] if "load_frequenz" in json_data else 1/30
            self.save_path = json_data["save_path"] if "save_path" in json_data else "/home/user/pictures"
            self.enable_save = "do_save" in json_data and json_data["do_save"]
            self.enable_load = "do_load" in json_data and json_data["do_load"]
            self.enable_debug = not "debug" in json_data or json_data["debug"]
            self.log(f"loaded config from {self.CONFIG_FILE}: {json_data}")
             
    def create_sub_pub(self):
        self.sub = [
            self.create_subscription(imgMsg, self.TOPIC["IMG"], self.save, 10),
            self.create_subscription(strMsg, self.TOPIC["SAVE_PATH"], self.set_save_path, 10),
            self.create_subscription(strMsg, self.TOPIC["LOAD_PATH"], self.set_load_path, 10),
            self.create_subscription(floatMsg, self.TOPIC["LOAD_FRQ"], self.set_load_frequenz, 10),
            self.create_subscription(boolMsg, self.TOPIC["DO_SAVE"], self.enable_saveing, 10),
            self.create_subscription(boolMsg, self.TOPIC["DO_LOAD"], self.enable_loading, 10),
            self.create_subscription(boolMsg, self.TOPIC["DEBUG"], self.enable_logging, 10)
        ]
        self.pub = {
            "IMG": self.create_publisher(imgMsg, self.TOPIC["IMG"], 10),
            "LOG": self.create_publisher(strMsg, self.TOPIC["LOG"], 10)
        }
    
    def set_load_frequenz(self, msg:floatMsg):
        self.load_frequenz = msg.data
        self.timer.cancel()
        self.timer = self.create_timer(self.load_frequenz, self.load)
        self.save_config()
        self.log(f"set load frequenz to {self.load_frequenz}")

    def set_save_path(self, msg:strMsg):
        self.save_path = msg.data
        self.save_config()
        self.log(f"set save path to {self.save_path}")
    
    def set_load_path(self, msg:strMsg):
        self.load_path = msg.data
        self.save_config()
        self.log(f"set load path to {self.load_path}")
        
    def enable_saveing(self, msg:boolMsg):
        self.enable_save = msg.data
        self.save_config()
        self.log(f"set do load to {self.enable_save}")
    
    def enable_loading(self, msg:boolMsg):
        self.enable_load = msg.data
        self.save_config()
        self.log(f"set do load to {self.enable_load}")
        
    def enable_logging(self, msg:boolMsg):
        self.enable_debug = msg.data
        self.save_config()
        self.log(f"set debug to {self.enable_debug}")
    
    def save(self, msg:imgMsg):
        if self.enable_save:
            img = self.bridge.imgmsg_to_cv2(msg)
            name = self.save_path + "/bild_" +  str(self.save_file_count) + ".jpg"
            success = cv2.imwrite(name, img)
            self.save_file_count += 1
            self.log(f"saved img to {name} " + "successfull" if success else "failed")
   
    def load(self):
        if self.enable_load:
            if not Path(self.load_path + f"/bild_{self.load_file_count}.jpg").exists():
                self.load_file_count = 1
            img = cv2.imread(self.load_path+f"/bild_{self.load_file_count}.jpg", cv2.IMREAD_GRAYSCALE)
            msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
            self.load_file_count += 1
            self.pub["IMG"].publish(msg)
            self.log(f"published img from {self.load_path}/bild_{self.load_file_count}.jpg")
    
    def log(self, log_text):
        if self.enable_debug:
            msg = strMsg()
            msg.data = f"{datetime.now()} [SaveNode]: {log_text}"
            self.pub["LOG"].publish(msg)
            print(msg.data)

def main(args=None):
    ros.init()
    ros.spin(SaveNode())
    ros.shutdown()

if __name__ == "__main__":
    main()
