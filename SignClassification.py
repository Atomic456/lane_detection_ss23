import cv2
import rclpy as ros
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Int32

from time import sleep

import numpy as np
import tensorflow as tf
from tensorflow import keras

class SignClassification(Node):
    
    def __init__(self):
        super().__init__("sign_classification")
        self.bridge = CvBridge()

        self.create_subscription(Image, "/perception/image_gray8", self.predict_sign, 10)

        self.speed_publisher = self.create_publisher(Float32, "/speed/speed", 10)

        self.labels = ['background', 'hardstop', 'speedlimit', 'speedlimitoff', 'stop']

        self.last_sign = -1
        self.last_event = -1
        
        self.current_threshold = 0
        self.sign_threshold = 1.3

        self.max_speed = 0.45
        self.current_speed = self.max_speed

        # load model
        model = tf.keras.models.load_model("./saved_model")

        model.summary()

        self.model = tf.keras.Sequential([
            model,
            tf.keras.layers.Softmax()
        ])

        self.model.build(input_shape=(None, 64, 64, 1))

        self.model.summary()

        self.speed_publish(self.current_speed)

    def predict_sign(self, img:Image):

        # rotieren und anzeigen des bildes
        input_image = self.bridge.imgmsg_to_cv2(img)
        input_image = cv2.rotate(input_image, cv2.ROTATE_180)
        cv2.imshow("Input", input_image)
         
        # für das Model, benötigt eine extra dimension warum auch immer
        input_image = np.expand_dims(input_image, axis=0)

        # das predicten an sich + das label mit höchster prediction extrahieren
        prediction = self.model.predict(input_image)
        label_index = tf.math.argmax(prediction, axis=-1)[0].numpy()

        label = self.labels[label_index]

        highest_predict = max(prediction[0])
        high_conf = " !! HIGH CONF !!" if highest_predict > 0.65 else ""

        if (high_conf != "" and label_index != 0):
            print("label: " + label + '\n\r' + "confidence: " + str(highest_predict) + high_conf)
            current_sign = int(label_index)
            if (self.last_sign == current_sign):
                self.current_threshold += highest_predict
            else:
                self.current_threshold = 0
            if (self.current_threshold >= self.sign_threshold): #and self.last_event != current_sign):
                self.sign_event(current_sign)
                self.last_event = current_sign

            self.last_sign = current_sign

        cv2.waitKey(1)

    def sign_event(self, label_index):
        print("received an event for:", self.labels[label_index])
        if label_index == 1: #hardstop
            self.stop_for_seconds(10)
        elif label_index == 2: #speedlimit
            self.current_speed = 0.3
            self.speed_publish(self.current_speed)
        elif label_index == 3: #speedlimitoff
            self.current_speed = self.max_speed
            self.speed_publish(self.current_speed)
        elif label_index == 4: #stop
            self.stop_for_seconds(3)
        sleep(3)

    def stop_for_seconds(self, seconds):
        print("now stopping for " + str(seconds) + " seconds")
        self.speed_publish(0.0)
        sleep(seconds)
        self.speed_publish(self.current_speed)


    def speed_publish(self, speed):
        print("now driving with " + str(speed) + " speed")
        msg = Float32()
        msg.data = speed
        self.speed_publisher.publish(msg)

def main(args=None):
    ros.init()
    ros.spin(SignClassification())
    ros.shutdown()

if __name__ == '__main__':
    main()
