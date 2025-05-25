from ultralytics import YOLO
import cv2
import numpy as np
import math

class resistor_detector:
    def __init__(self, model_path="resistor_detector_model.pt", camera_index=0,
                 screen_width_m=0.1, screen_height_m=0.1):  # 10cm default
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(camera_index)

        # Read one frame to determine resolution and center
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Could not read from camera.")

        self.frame_width_px = frame.shape[1]
        self.frame_height_px = frame.shape[0]

        self.frame_center_x = self.frame_width_px / 2
        self.frame_center_y = self.frame_height_px / 2

        # Calculate mm per pixel based on screen dimensions
        self.mm_per_pixel_x = screen_width_m / self.frame_width_px
        self.mm_per_pixel_y = screen_height_m / self.frame_height_px

    def get_angle_and_draw_line(self, binary_img, x1, y1, x2, y2, frame, is_vertical):
        points = cv2.findNonZero(binary_img)
        if points is not None and len(points) > 1:
            points = sorted(points, key=lambda pt: pt[0][1] if is_vertical else pt[0][0])
            point1, point2 = tuple(points[0][0]), tuple(points[-1][0])
            point1_offset = (point1[0] + x1, point1[1] + y1)
            point2_offset = (point2[0] + x1, point2[1] + y1)

            # Draw line
            cv2.line(frame, point1_offset, point2_offset, (0, 0, 255), 2)

            # Calculate angle
            delta_x = point2_offset[0] - point1_offset[0]
            delta_y = point2_offset[1] - point1_offset[1]
            raw_angle_deg = math.degrees(math.atan2(delta_y, delta_x))

            # Convert so 0° = north (up), range [-180, 180]
            angle_deg = 90 - raw_angle_deg
            if angle_deg > 180:
                angle_deg -= 360
            elif angle_deg < -180:
                angle_deg += 360

            return -angle_deg
        return None

    def find_resistors(self, draw=False):
        ret, frame = self.cap.read()
        if not ret:
            return None

        results = self.model(frame)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()
                if confidence > 0.5:
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    # Offset from center in pixels
                    rel_x_px = center_x - self.frame_center_x
                    rel_y_px = center_y - self.frame_center_y

                    # Convert to mm, invert Y-axis
                    rel_x_mm = rel_x_px * self.mm_per_pixel_x
                    rel_y_mm = -rel_y_px * self.mm_per_pixel_y

                    # Extract ROI for angle detection
                    roi = frame[y1:y2, x1:x2]
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    _, binary_roi = cv2.threshold(gray_roi, 50, 255, cv2.THRESH_BINARY_INV)

                    # Check if shape is vertical or horizontal
                    is_vertical = (y2 - y1) > (x2 - x1)
                    angle_deg = self.get_angle_and_draw_line(binary_roi, x1, y1, x2, y2, frame, is_vertical)
                    if angle_deg is None:
                        angle_deg = 0.0

                    if draw:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"x: {rel_x_mm:.5f}mm, y: {rel_y_mm:.5f}mm, θ: {angle_deg:.1f}°"
                        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_ITALIC, 0.5, (0, 255, 0), 2)
                        cv2.imshow("Resistor Detection", frame)
                        cv2.waitKey(1)

                    return (rel_x_mm, rel_y_mm, angle_deg)

        if draw:
            cv2.imshow("Resistor Detection", frame)
            cv2.waitKey(1)

        return None

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

