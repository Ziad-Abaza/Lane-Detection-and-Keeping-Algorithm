import cv2
import numpy as np
import math

class LaneDetection:
    def __init__(self, video_path):
        self.video = cv2.VideoCapture(video_path)
        self.speed = 8
        self.kp = 0.4
        self.kd = self.kp * 0.65

    def detect_edges(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)  
        return edges

    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)
        vertices = np.array([[(0, height), (width/3, height/2), (width*2/3, height/2), (width, height)]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges

    def detect_line_segments(self, cropped_edges):
        rho = 1  
        theta = np.pi / 180  
        min_threshold = 30  
        line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, np.array([]), minLineLength=5, maxLineGap=150)
        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        lane_lines = []
        
        if line_segments is None:
            print("no line segments detected")
            return lane_lines

        height, width,_ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary
        
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    print("skipping vertical lines (slope = infinity")
                    continue
                
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - (slope * x1)
                
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        return lane_lines

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height 
        y2 = int(y1 / 2)  
        if slope == 0:
            slope = 0.1
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return [[x1, y1, x2, y2]]

    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=6):
        line_image = np.zeros_like(frame)
        
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    if length < 780:
                        line_color = (0, 255, 0)  
                    else:
                        line_color = (0, 255, 255)  
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                    
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        
        return line_image

    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)
        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
        return heading_image

    def get_steering_angle(self, frame, lane_lines):
        height, width,_ = frame.shape
        
        if len(lane_lines) == 2:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            mid = int(width / 2)
            x_offset = (left_x2 + right_x2) / 2 - mid
            y_offset = int(height / 2)
            
        elif len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
            
        elif len(lane_lines) == 0:
            x_offset = 0
            y_offset = int(height / 2)
            
        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        steering_angle = angle_to_mid_deg + 90

        return steering_angle

    def run(self):
        while True:
            ret, frame = self.video.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            edges = self.detect_edges(frame)
            roi = self.region_of_interest(edges)
            line_segments = self.detect_line_segments(roi)
            lane_lines = self.average_slope_intercept(frame, line_segments)
            lane_lines_image = self.display_lines(frame, lane_lines)
            steering_angle = self.get_steering_angle(frame, lane_lines)
            heading_image = self.display_heading_line(lane_lines_image, steering_angle)
            
            cv2.imshow("heading line", heading_image)
            
            key = cv2.waitKey(1)
            if key == 32:
                break

        self.video.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ld = LaneDetection("video/test3.mp4")
    ld.run()
