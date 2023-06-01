import cv2
import rclpy as ros
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import tensorflow as tf
from tensorflow import keras

class SignClassification(Node):
    
    def __init__(self):
        super().__init__("sign_classification")
        self.bridge = CvBridge()
        self.create_subscription(Image, "/perception/image_gray8", self.predict_sign, 10)  

        self.labels = ['background', 'hardstop', 'speedlimit', 'speedlimitoff', 'stop']

        # load model
        model = tf.keras.models.load_model("/media/sf_lane_detection_ss23/saved_model")

        model.summary()

        self.model = tf.keras.Sequential([
            model,
            tf.keras.layers.Softmax()
        ])

        self.model.build(input_shape=(None, 64, 64, 3))

        self.model.summary()

    def predict_sign(self, img:Image):
        #print("got an image")

        input_image = self.bridge.imgmsg_to_cv2(img)
        input_image = cv2.cvtColor(input_image, cv2.COLOR_GRAY2BGR)
        input_image = cv2.rotate(input_image, cv2.ROTATE_180)
        cv2.imshow("original", input_image)

        #cv2.imshow("visualisation", visualisation_img)
        
        resized_image = tf.image.resize(
            input_image,
            tf.constant([64, 64])
        )

        resized_image = np.expand_dims(resized_image, axis=0)

        prediction = self.model(resized_image)
        label_index = tf.math.argmax(prediction, axis=1)[0].numpy()

        label = self.labels[label_index]
        highest_predict = max(prediction[0].numpy())
        #if (label != 'background' and highest_predict > 0.9):
        print("label: " + label + '\n\r' + "confidence: " + str(highest_predict))

        cv2.waitKey(1)


def main(args=None):
    ros.init()
    ros.spin(SignClassification())
    ros.shutdown()

if __name__ == '__main__':
    main()
