import cv2
import rclpy as ros
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Image

class LaneKeep(Node):

    def __init__(self):
        super().__init__("lane_keep")
        self.bridge = CvBridge()
    
    def houghLines(self, maskedImage):
        min_line_length = self.hight / 5
        max_line_gap = self.width / 6
        rho = 1
        theta = np.py / 180
        hough_threshold = 13
        lines = cv2.HoughLinesP(maskedImage, rho, theta, hough_threshold, np.array([]), minLineLength = min_line_length, maxLineGap = max_line_gap)
        return lines

    def calculateLineSlope(self, lane_lines):
        slopes = []
        res_lines = []
        for line in lane_lines:
            x1, y1, x2, y2 = [0,0,0,0]
            x1, y1, x2, y2 = line.reshape(4)

            if x1 == x2:
                slope = float(99999) # not able to calculate slope
            elif x1 < x2:
               slope = float((y2 - y1)) / float((x2 - x1))
               slopes.append(slope)
               res_lines.append(line)
            else:
                slope = float((y1 - y2)) / float((x1 - x2))
                slopes.append(slope)
                res_lines.append(line)
        
        return res_lines, slopes
        
    def sortLines(image_center, detected_lines, line_slopes):

        right_lines = []
        left_lines = []

        min_slope_thres = 0.3
        max_slope_thres = 12
        border_offset = 0.25

        for i in range(len(line_slopes)):
            x1, y1, x2, y2 = detected_lines[i]

            # Right lane
            if (line_slopes[i] < min_slope_thres) and (min(x1, x2) > (1 - border_offset) * image_center) and (line_slopes[i] < max_slope_thres):
                right_lines.append(detected_lines[i])
            # Left lane
            elif (line_slopes[i] < -min_slope_thres) and (max(x1, x2) < (1 + border_offset) * image_center) and (line_slopes[i] > -max_slope_thres):
                left_lines.append(detected_lines[i])
        
        return left_lines, right_lines
    
    def linearFit(lane_lines):
        slopes = []
        x_intersects = []

        if len(lane_lines) > 0:
            for line in lane_lines:
                x1, y1, x2, y2 = line
                line_m = float(y2 - y1) / float(x2 - x1)
                line_b = y1 - line_m * x1
                x_intersect = -line_b / line_m
                slopes.append(line_m)
                x_intersects.append(x_intersect)

            # avrage slope of all lines
            m = sum(slopes) / len(slopes)

            # avrage x intersect of all lines
            avg_x_intersect = sum(x_intersects) / len(x_intersects)

            b = -m * avg_x_intersect

            line_found = True
            return m, b, line_found
        else:
            line_found = False
            m, b = 0, 0 # set default values for the return
            return m, b, line_found


    def laneKeep(self, img:Image):
        """read Image and convert for further processing"""
        # convert image to cv2
        gray_scale_img  = self.bridge.imgmsg_to_cv2(img)

        # safe image size
        self.height, self.width, self.channels = gray_scale_img.shape
        
        # blur image to reduce noise
        blured_img = cv2.GaussianBlur(gray_scale_img, (3,3), 0)

        # image center
        self.img_center = round(self.width / 2)

        """Image processing before detecting lanes"""
        # define region of interest
        region_of_interest = [(15,10),(15,300),(240,0),(240,320)]

        # mask off the image
        image_mask = np.zeros_like(gray_scale_img)
        image_mask = cv2.fillPoly(image_mask, region_of_interest, 255)
        masked_img = cv2.bitwise_and(edges, image_mask)
        
        # detecting edges in the image
        edges = cv2.Canny(masked_img, 252, 255)

        """Detect lines in pre processed image"""
        # detecting lines in the image
        lines = self.houghLines(edges)

        # calculate slope of detected lines
        relevant_lines, slopes = self.calculateLineSlope(lines)

        # split lines into right and left lanes
        left_lines, right_lines = self.sortLines(self.img_center, relevant_lines, slopes)
        
        right_line_m, right_line_b, right_line_found = self.linearFit(right_lines)
        left_line_m, left_line_b, left_line_found = self.linearFit(left_lines)

        """Calculate steering values"""
        if right_line_found:
            right_y2 = self.height - 1
            right_x2 = int((right_y2 - right_line_b) / right_line_m)

        if left_line_found:
            left_y2 = self.height - 1 
            left_x2 = int((left_y2 - left_line_b) / left_line_m)
        
        lane_center = (right_x2 + left_x2) / 2
        lane_width = abs(right_x2 - left_x2)
        car_position = -self.img_center + lane_center

        steering_value = (abs(car_position) / lane_width) * (car_position / abs(car_position))

def main(args=None):
    ros.init()
    ros.spin(LaneKeep())
    ros.shutdown()

if __name__ == '__main__':
    main()



        

