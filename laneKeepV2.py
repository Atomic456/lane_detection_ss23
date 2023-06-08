import cv2
import rclpy as ros
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import Image as Image
from math import atan
from math import sqrt



class LaneKeep(Node):

    def __init__(self):
        super().__init__("lane_keep")
        self.bridge = CvBridge()
        self.steering_publisher = self.create_publisher(Float32, "/pid/steering", 10)
        self.create_subscription(Image, "/perception/image_gray8", self.laneKeep, 10)
        self.lane_width = 0
        self.steering_value = 0

    def line_visualisation(self, img, lane_lines):
        if lane_lines is not None:
            for point_array in lane_lines:
                x1,y1,x2,y2 = point_array[0]
                cv2.line(img, (x1,y1), (x2,y2), (0,255,0), 2)
        return img

    def end_visualisation(self, img, steering_value, region_of_interest):
        height, width, _ = img.shape
        width_value = int(np.interp(steering_value, [-1.0,1.0], [0,width]))
        cv2.circle(img, (width_value,0), 5, (0,0,255), 20)
        cv2.putText(img, "{:.2f}".format(steering_value), (width_value,40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 1)

        for poly in region_of_interest:
            poly = poly.reshape((-1, 1, 2))
            cv2.polylines(img, [poly], True, (255,0,0), 1)
        return img

    def publishSteeringValue(self, steering_value):
        # cap seering to a max value of 0.8/-0.8
        # create steering message
        msg = Float32()
        msg.data = np.clip(steering_value, -0.8, 0.8)
        
        # publish message to car
        self.steering_publisher.publish(msg)

    def localisation(self, left_x, right_x):
        #calculate relevant positione of the lane and the car
        lane_center = (right_x + left_x) / 2
        car_position = -self.img_center + lane_center

        #return steering position
        if self.lane_width != 0:
            return car_position/(self.lane_width/2)

    def houghLines(self, masked_Image):
        """
        min_line_length = self.height / 7
        max_line_gap = self.width / 6
        rho = 1
        theta = np.pi / 180
        hough_threshold = 12
        lines = cv2.HoughLinesP(masked_Image, rho, theta, hough_threshold, np.array([]), minLineLength = min_line_length, maxLineGap = max_line_gap)
        return lines
        """
        min_line_length = 17
        max_line_gap = 8
        rho = 2
        theta = np.pi / 180
        hough_threshold = 13
        lines = cv2.HoughLinesP(masked_Image, rho, theta, hough_threshold, np.array([]), minLineLength = min_line_length, maxLineGap = max_line_gap)
        return lines
        

    def calculateLineSlope(self, lane_lines):
        slopes = []
        res_lines = []
        if lane_lines is None:
            return res_lines, slopes
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
        
    def sortLines(self, image_center, detected_lines, line_slopes):

        right_lines = []
        left_lines = []

        min_slope_thres = 0.1
        max_slope_thres = 18
        border_offset = 0.25

        for i in range(len(line_slopes)):
            x1, y1, x2, y2 = detected_lines[i].reshape(4)

            # Right lane
            if (line_slopes[i] > min_slope_thres) and (min(x1, x2) > (1 - border_offset) * image_center) and (line_slopes[i] < max_slope_thres):
                right_lines.append(detected_lines[i])
            # Left lane
            elif (line_slopes[i] < -min_slope_thres) and (max(x1, x2) < (1 + border_offset) * image_center) and (line_slopes[i] > -max_slope_thres):
                left_lines.append(detected_lines[i])
        
        return left_lines, right_lines
    
    def linearFit(self, lane_lines):
        slopes = []
        x_intersects = []

        if len(lane_lines) > 0:
            for line in lane_lines:
                x1, y1, x2, y2 = line.reshape(4)
                line_m = float(y2 - y1) / float(x2 - x1)
                line_b = y1 - line_m * x1
                x_intersect = -line_b / line_m
                slopes.append(line_m)
                x_intersects.append(x_intersect)

            # avrage slope of all lines
            if len(slopes) != 0:
                m = sum(slopes) / len(slopes)
            else:
                m = 0

            # avrage x intersect of all lines
            if len(x_intersects) != 0:
                avg_x_intersect = sum(x_intersects) / len(x_intersects)
            else:
                avg_x_intersect = 0

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
        gray_scale_img = cv2.rotate(gray_scale_img, cv2.ROTATE_180)
        # safe image size
        self.height, self.width = gray_scale_img.shape
        
        # blur image to reduce noise
        blurred_img = cv2.GaussianBlur(gray_scale_img, (3,3), 0)

        # image center
        self.img_center = round(self.width / 2)

        """Image processing before detecting lanes"""
        # define region of interest
        region_of_interest = np.array([[[(0,50),(0,240),(320,240),(320,50)]]])
        
        # detecting edges in the image
        edges = cv2.Canny(blurred_img, 252, 255)
        
        # mask off the image
        image_mask = np.zeros_like(gray_scale_img)
        image_mask = cv2.fillPoly(image_mask, region_of_interest, 255)
        masked_img = cv2.bitwise_and(edges, image_mask)

        """Detect lines in pre processed image"""
        # detecting lines in the image
        lines = self.houghLines(masked_img)

        # calculate slope of detected lines
        relevant_lines, slopes = self.calculateLineSlope(lines)

        # split lines into right and left lanes
        left_lines, right_lines = self.sortLines(self.img_center, relevant_lines, slopes)
        
        right_line_m, right_line_b, right_line_found = self.linearFit(right_lines)
        left_line_m, left_line_b, left_line_found = self.linearFit(left_lines)
        
        steering_value = 0
        """Calculate steering"""
        #Clculate steering values with both lines
        if right_line_found and left_line_found:
            right_y = self.height - 20
            if right_line_m != 0:
                right_x = int((right_y - right_line_b) / right_line_m)
            left_y = self.height - 20
            if left_line_m != 0:
                left_x2 = int((left_y - left_line_b) / left_line_m)
        
            steering_value = self.localisation(left_x2, right_x)

        #Calculate steering values with only the right line
        elif right_line_found and not left_line_found:
            #calculate positon of right line
            right_y = self.height - 20
            if right_line_m != 0:
                right_x = int((right_y - right_line_b) / right_line_m)

            #aprocimate positon of left line
            left_x2 = right_x - self.lane_width

            steering_value = self.localisation(left_x2, right_x)

        #Calculate steering values with only the left line
        elif left_line_found and not right_line_found:
            #calculate positon of right line
            left_y = self.height - 20
            if left_line_m != 0:
                left_x2 = int((left_y - left_line_b) / left_line_m)

            #aprocimate positon of left line
            right_x = left_x2 + self.lane_width

            steering_value = self.localisation(left_x2, right_x)

        visualisation_img = cv2.cvtColor(gray_scale_img, cv2.COLOR_GRAY2BGR)

        
               
        # send steering value
        print(steering_value)
        self.publishSteeringValue(steering_value)
        # visualisation        
        visualisation_img = self.line_visualisation(visualisation_img, lines)
        visualisation_img = self.end_visualisation(visualisation_img, steering_value, region_of_interest)

        cv2.imshow("Visualisation", visualisation_img)
        cv2.imshow("Masked Image", masked_img)
        cv2.waitKey(1)

def main(args=None):
    ros.init()
    ros.spin(LaneKeep())
    ros.shutdown()

if __name__ == '__main__':
    main()



        

