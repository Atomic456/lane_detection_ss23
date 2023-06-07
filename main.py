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
from math import atan

class LanePrediction(Node):
    
    def __init__(self):
            super().__init__("lane_prediction")
            self.bridge = CvBridge()
            self.steering_publisher = self.create_publisher(Float32, "/steering/steering", 10)
            self.create_subscription(Image, "/perception/image_gray8", self.e2e_steering, 10)
            self.create_subscription(Float32, "/steering/steering", self.set_steering_out, 10)
            self.steering_out = 0.0

    def set_steering_out(self, msg:Float32):
        self.steering_out = msg.data  
    
    def calc_steeringangle(self, lane_oriantation):
        if lane_oriantation > 319:
            lane_oriantation = 319
        if lane_oriantation < 0:
            lane_oriantation = 0
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
    
    def calc_single_steeringangle(self, angle):
        steering_input = 0
        steering_input = angle/90
        return max(min(1.0, steering_input), -1.0)

    def line_visualisation(self, img, lane_lines):
        for point_array in lane_lines:
            x1,y1,x2,y2 = point_array[0]
            cv2.line(img, (x1,y1), (x2,y2), (0,255,0), 2)
        return img

    def end_visualisation(self, img, steering_value, region_of_interest):
        height, width, _ = img.shape
        width_value = int(np.interp(steering_value, [-1.0,1.0], [0,width]))
        cv2.circle(img, (width_value,0), 5, (0,0,255), 20)
        cv2.putText(img, "{:.2f}".format(steering_value), (width_value,40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 1)

        width_value = int(np.interp(self.steering_out, [-1.0,1.0], [0,width]))
        cv2.circle(img, (width_value,50), 3, (255,255,0), 5)
        cv2.putText(img, "{:.2f}".format(self.steering_out), (width_value,90), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,0), 1)

        for poly in region_of_interest:
            poly = poly.reshape((-1, 1, 2))
            cv2.polylines(img, [poly], True, (255,0,0), 1)
        return img

    def e2e_steering(self, img:Image):
        print("got an image")

        input_image = self.bridge.imgmsg_to_cv2(img)
        gray_scale_copy = cv2.rotate(input_image, cv2.ROTATE_180)
        # cv2.imshow("original", gray_scale_copy)

        # Noise reduction
        blured_img = cv2.GaussianBlur(gray_scale_copy, (3,3), 0)
        # cv2.imshow("blured", blured_img)

        # Convert gray scale image to binary image
        threshold, binary_img = cv2.threshold(blured_img, 254, 255, cv2.THRESH_BINARY)
        # cv2.imshow("binary", binary_img)

        # Run edge detection
        edge_detection_img = cv2.Canny(binary_img, 200, 255)

        # Define region of interrest        
        region_of_interest = np.array([
            [[(0,0),(0,240),(319,240),(319,0)]]
        ])

        visualisation_img = cv2.cvtColor(gray_scale_copy, cv2.COLOR_GRAY2BGR)

        i = 0
        steering_angles = []
        for cycle in region_of_interest:
            # Apply region of interest to mask of the image
            image_mask = np.zeros_like(gray_scale_copy)
            image_region_interrest = cv2.fillPoly(image_mask, cycle, 255)
            masked_of_image = cv2.bitwise_and(edge_detection_img, image_region_interrest)
            #window_name = "masked_image" + str(i)
            # cv2.imshow(window_name, masked_of_image)

            # extrackt streight lines
            lane_lines = cv2.HoughLinesP(masked_of_image, rho=2, theta=np.pi/180, threshold=13, lines=np.array([]), minLineLength=18, maxLineGap=8)

            left_lane_line = []
            right_lane_line = []
            if lane_lines is not None:
                visualisation_img = self.line_visualisation(visualisation_img, lane_lines)
                for line in lane_lines:
                    x1, y1, x2, y2 = [0,0,0,0]
                    x1, y1, x2, y2 = line.reshape(4)
                    line_parameters = np.polyfit([x1,x2], [y1,y2], 1)
                    m, b = line_parameters
                    height, width = gray_scale_copy.shape
                    x = (240-b)/m
                    if(x > 160):
                        right_lane_line.append((m,b))
                    else:
                        left_lane_line.append((m,b))
            
            if len(left_lane_line) > 0 and len(right_lane_line) == 0:
                print("left_lane", end='')
                avr_lane_line_paramters = np.average(left_lane_line, axis=0)
                #-----------------------------------------------------------------------------------------------
                #line_l = self.calc_coordinates(avr_lane_line_paramters)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                m_i, b_i = avr_lane_line_paramters
                roi_angle = atan(m_i)
                roi_angle = (roi_angle * 180)/ np.pi
                if roi_angle > 0:
                    roi_angle = -90 + roi_angle
                else:
                    roi_angle = 90 + roi_angle
                steering_angles.append(self.calc_single_steeringangle(roi_angle))
            elif len(right_lane_line) > 0 and len(left_lane_line) == 0:
                print("right lane", end='')
                avr_lane_line_paramters = np.average(right_lane_line, axis=0)
                #-----------------------------------------------------------------------------------------------
                #line_r = self.calc_coordinates(avr_lane_line_paramters)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                m_i, b_i = avr_lane_line_paramters
                roi_angle = atan(m_i)
                roi_angle = (roi_angle * 180)/ np.pi
                if roi_angle > 0:
                    roi_angle = -90 + roi_angle
                else:
                    roi_angle = 90 + roi_angle
                steering_angles.append(self.calc_single_steeringangle(roi_angle))
            elif len(left_lane_line) > 0 and len(right_lane_line) > 0:
                print("both lanes", end='')
                left_lane_line_avr = np.average(left_lane_line, axis = 0)
                #-----------------------------------------------------------------------------------------------
                #line_l = self.calc_coordinates(left_lane_line_avr)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                right_lane_line_avr = np.average(right_lane_line, axis = 0)
                #-----------------------------------------------------------------------------------------------
                #line_r = self.calc_coordinates(right_lane_line_avr)
                #cv2.line(gray_scale_copy, (line_l[0], line_l[1]), (line_l[2], line_l[3]), (0,255,2), 3)
                #-----------------------------------------------------------------------------------------------
                lane_direction = self.calc_lane_direction(left_lane_line_avr, right_lane_line_avr)
                steering_angle = self.calc_steeringangle(lane_direction)
                steering_angles.append(steering_angle)
                print(f"{lane_direction} -> steeringangle {i}")
            else:
                steering_angles.append(-100)

            #image_name = "line" + str(i)
            i = i + 1
            #cv2.imshow(image_name, gray_scale_copy)

        print(steering_angles)
        steering_input = 0

        angle = steering_angles[0]
        if angle != -100:
             steering_input = angle

        msg = Float32()
        msg.data = steering_input / 80
        print(msg.data)

        visualisation_img = self.end_visualisation(visualisation_img, msg.data, region_of_interest)
        cv2.imshow("visualisation", visualisation_img)

        self.steering_publisher.publish(msg)
        cv2.waitKey(1)


def main(args=None):
    ros.init()
    ros.spin(LanePrediction())
    ros.shutdown()

if __name__ == '__main__':
    main()
