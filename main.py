import cv2
import rclpy as ros
from rclpy.node import Node
from lic import lic as lic
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import Image as Image
from cv_bridge import CvBridge
import numpy as np
from datetime import date

class SaveNode(Node):
    def __init__(self):
        super().__init__("SaveNode")
        self.bridge = CvBridge()
        self.imageSub = self.create_subscription(Image, "/perception/image_gray8", self.save, 10)
        self.count = 1
        
    def save(self, msg:Image):
        image = self.bridge.imgmsg_to_cv2(msg)
        name = "/home/user/pictures/bild_" +  str(self.count) + ".jpg"
        success = cv2.imwrite(name, image)
        self.count = self.count + 1
        print(f"saving image to {name} successfull" if success else f"saving image to {name} failed")

class KeyControl(Node):
    def __init__(self):
        super().__init__("KeyControl")
        self.bridge = CvBridge()
        self.imageSub = self.create_subscription(Image, "/perception/image_gray8", self.show, 10)
        self.speed_publisher = self.create_publisher(Float32, "/speed/speed", 10)
        self.steering_publisher = self.create_publisher(Float32, "/steering/steering", 10)
        self.wasdControl = False
        self.emergencyDisable = False

    def wasd_reset(self):
        self.wasd_speed = 0.0
        self.wasd_steering = 0.0

        self.speed_publisher.publish(Float32())
        self.steering_publisher.publish(Float32())

    def clamp(self, n, min, max):
        if n > max:
            return max
        elif n < min:
            return min
        else:
            return n


    def show(self, msg:Image):
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("Bild-Autofahren", image)
        while True:
            key = cv2.pollKey()
            speed_msg = Float32()
            steering_msg = Float32()

            # leerzeichen für not-aus
            if (key == 32):
                self.emergencyDisable = not self.emergencyDisable
                self.wasd_reset()
                print("emergencyDisable is now " + str(self.emergencyDisable))

            # X für wasd-Steuerung an und aus
            if (key != -1 and (chr(key) == 'x' or chr(key) == 'X')):
                self.wasdControl = not self.wasdControl
                self.wasd_reset()
                print("wasdControl is now " + str(self.wasdControl))

            if (self.wasdControl is False or self.emergencyDisable is True):
                return

            if (key != -1):
                match (chr(key)):
                    case 'w'|'W': 
                        self.wasd_speed += 0.15
                    case 's'|'S':
                        self.wasd_speed -= 0.15
                    case 'a'|'A':
                        self.wasd_steering -= 0.2
                    case 'd'|'D':
                        self.wasd_steering += 0.2

                self.wasd_speed = self.clamp(self.wasd_speed, -1.0, 1.0)
                self.wasd_steering = self.clamp(self.wasd_steering, -0.8, 0.8)

                speed_msg.data = self.wasd_speed
                steering_msg.data = self.wasd_steering
                self.speed_publisher.publish(speed_msg) #damit ist speed ein toggle
                self.steering_publisher.publish(steering_msg) #damti ist steering ein toggle

                print("Input received: " + str(key) + ", message sent: Speed: " + str(speed_msg.data) + ", Steering: " + str(steering_msg.data))   

class LanePrediction(Node):
    
    def __init__(self):
            super().__init__("lane_prediction")
            self.bridge = CvBridge()
            self.steering_publisher = self.create_publisher(Float32, "/steering/steering", 10)
            self.create_subscription(Image, "/perception/image_gray8", self.e2e_steering, 10)

    
    
    def calc_steeringangle(self, lane_oriantation):
        direction = lane_oriantation - 160
        steering_input = direction / 160
        return steering_input

    def calc_single_lane_direction(self, avr_lane_paramters):
        m, b = avr_lane_paramters
        x_intersect = (0-b)/m
        return int(round(x_intersect))
    
    def calc_lane_direction(self, avr_left_lane_parmeters, avr_right_lane_paramters):
        m_l, b_l = avr_left_lane_parmeters
        m_r, b_r = avr_right_lane_paramters
        x_intersect = (b_r-b_l)/(m_l-m_r)
        return int(round(x_intersect))
    
    def calc_coordinates(self, line_parameters):
        m, b = line_parameters
        y1 = 165
        y2 = 85
        x1 = int((y1-b)/m)
        x2 = int((y2-b)/m)
        return np.array([x1, y1, x2, y2])


    def e2e_steering(self, img:Image):
        #-------------------------- Variables for local testing --------------------------
        #input_image = cv2.imread("/home/atomic/img/lane3.jpg")
        #gray_scale_copy = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        #---------------------------------------------------------------------------------

        print("got an image")

        input_image = self.bridge.imgmsg_to_cv2(img)
        gray_scale_copy = input_image
#        cv2.imshow(gray_scale_copy)

        # Noise reduction
        blured_img = cv2.GaussianBlur(gray_scale_copy, (3,3), 0)

        # Convert gray scale image to binary image
        threshold, binary_img = cv2.threshold(blured_img, 235, 255, cv2.THRESH_BINARY)
        
        # Run edge detection
        edge_detection_img = cv2.Canny(binary_img, 200, 255)

        # Define region of interrest        
        region_of_interest = np.array([
            [[(0,155),(0,205),(319,205),(319,155)]],
            [[(0,105),(0,155),(319,155),(319,105)]],
            [[(0,65),(0,105),(319,105),(319,65)]]
        ])
    
        steering_angles = []
        steerin_wights = [40,45,15]
        
        i = 0
        for cycle in region_of_interest:
            # Apply region of interest to mask of the image
            image_mask = np.zeros_like(gray_scale_copy)
            image_region_interrest = cv2.fillPoly(image_mask, cycle, 255)
            masked_of_image = cv2.bitwise_and(edge_detection_img, image_region_interrest)
            window_name = "masked_image" + str(i)
            #cv2.imshow(window_name, masked_of_image)

            # extrackt stright lines
            lane_lines = cv2.HoughLinesP(masked_of_image, rho=2, theta=np.pi/180, threshold=13, lines=np.array([]), minLineLength=18, maxLineGap=8)

            left_lane_line = []
            right_lane_line = []
            if lane_lines is not None:
                for line in lane_lines:
                    x1, y1, x2, y2 = line.reshape(4)
                    line_parameters = np.polyfit((x1,x2), (y1,y2), 1)
                    m, b = line_parameters
                    if(m < 0):
                        left_lane_line.append((m,b))
                    else:
                        right_lane_line.append((m,b))
            
            if len(left_lane_line) > 0 and len(right_lane_line) == 0:
                print("left_lane")
                avr_lane_line_paramters = np.average(left_lane_line, axis=0)
                #-----------------------------------------------------------------------------------------------
                #line_l = self.calc_coordinates(avr_lane_line_paramters)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                lane_direction = self.calc_single_lane_direction(avr_lane_line_paramters)
                steering_angle = self.calc_steeringangle(lane_direction)
                steering_angles.append(steering_angle)
                print(lane_direction, "-> steeringangle " + str(i))
            elif len(right_lane_line) > 0 and len(right_lane_line) == 0:
                print("right lane")
                avr_lane_line_paramters = np.average(right_lane_line, axis=0)
                #-----------------------------------------------------------------------------------------------
                #line_l = self.calc_coordinates(avr_lane_line_paramters)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                lane_direction = self.calc_single_lane_direction(avr_lane_line_paramters)
                steering_angle = self.calc_steeringangle(lane_direction)
                steering_angles.append(steering_angle)
                print(lane_direction, "-> steeringangle " + str(i))
            elif len(left_lane_line) > 0 and len(right_lane_line) > 0:
                print("both lanes")
                left_lane_line_avr = np.average(left_lane_line, axis = 0)
                #-----------------------------------------------------------------------------------------------
                #line_l = self.calc_coordinates(left_lane_line_avr)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                right_lane_line_avr = np.average(right_lane_line, axis = 0)
                #-----------------------------------------------------------------------------------------------
                #line_l = self.calc_coordinates(right_lane_line_avr)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                lane_direction = self.calc_lane_direction(left_lane_line_avr, right_lane_line_avr)
                steering_angle = self.calc_steeringangle(lane_direction)
                steering_angles.append(steering_angle)
                print(lane_direction, "-> steeringangle " + str(i))
            else:
                steering_angles.append(-100)

            #image_name = "line" + str(i)
            i = i + 1
            #cv2.imshow(image_name, gray_scale_copy)

        print(steering_angles)
        steering_input = 0
        div = 1
        for i in range(2):
            angle = steering_angles[i]
            if angle != -100:
                steering_input += angle * steerin_wights[i]
                div += steerin_wights[i]
    
        msg = Float32()
        if div != 1:
            div -= 1
        msg.data = steering_input / div
        print(msg.data)
        self.steering_publisher.publish(msg)


def main(args=None):
    ros.init()
#    ros.spin(KeyControl())
#    ros.spin(SaveNode())
    ros.spin(LanePrediction())
    ros.shutdown()

if __name__ == '__main__':
    main()
