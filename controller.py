# ros
import rclpy as ros
from rclpy.node import Node

# ros_msg
from std_msgs.msg import Bool as boolMsg
from std_msgs.msg import String as strMsg
from std_msgs.msg import Float32 as floatMsg

# others
from datetime import datetime

class Controller(Node):
    TOPIC = {
        "LOG": "/log/nodes/controller",
        "SAVE_PATH": "/config/nodes/save_node/save_path",
        "LOAD_PATH": "/config/nodes/save_node/load_path",
        "DO_SAVE": "/config/nodes/save_node/enable_saveing",
        "DO_LOAD": "/config/nodes/save_node/enabe_loading",
        "LOAD_FRQ": "/config/nodes/save_node/load_frequenz",
        "DEBUG": "/config/nodes/save_node/enable_logging"
    }
    
    def __init__(self):
        super().__init__("SaveNode")
        self.create_sub_pub()
        self.log("initilized")

    def create_sub_pub(self):
        self.pub = {
            "LOG": self.create_publisher(strMsg, self.TOPIC["LOG"], 10),
            "SAVE_PATH": self.create_publisher(strMsg, self.TOPIC["SAVE_PATH"], 10),
            "LOAD_PATH": self.create_publisher(strMsg, self.TOPIC["LOAD_PATH"], 10),
            "DO_SAVE": self.create_publisher(boolMsg, self.TOPIC["DO_SAVE"], 10),
            "DO_LOAD": self.create_publisher(boolMsg, self.TOPIC["DO_LOAD"], 10),
            "LOAD_FRQ": self.create_publisher(floatMsg, self.TOPIC["LOAD_FRQ"], 10),
            "DEBUG": self.create_publisher(boolMsg, self.TOPIC["DEBUG"], 10),
        }
    
    def log(self, log_text):
        msg = strMsg()
        msg.data = f"{datetime.now()} [SaveNode]: {log_text}"
        self.pub["LOG"].publish(msg)
        print(msg.data)
    
    def run(self):
        while ros.ok():
            selection = input("""\
What do you wanna do?
SAVE_PATH: set the path for images to be stored to
LOAD_PATH: set the path where the image is published from
DO_SAVE:   enables image saveing
DO_LOAD:   enables image publishing for testing issues
LOAD_FRQ:  set load frequenz
DEBUG:     enables debug prints\n""")
            if selection in ["SAVE_PATH", "LOAD_PATH"]:
                msg = strMsg()
                msg.data = input("What is the path? ")
                self.pub[selection].publish(msg)
            elif selection == "LOAD_FRQ":
                msg = floatMsg()
                msg.data = float(input("Please enter the frequenz as float "))
                self.pub[selection].publish(msg)
            else:
                msg = boolMsg()
                msg.data = input("Enable ? (y/n)") == 'y'
                self.pub[selection].publish(msg)

def main(args=None):
    ros.init()
    node = Controller()
    node.run()
    node.destroy_node()
    ros.shutdown()

if __name__ == "__main__":
    main()
