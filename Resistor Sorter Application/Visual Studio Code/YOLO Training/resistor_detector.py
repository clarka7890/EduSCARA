from ultralytics import YOLO
import cv2
import numpy as np
import math

# Load the trained YOLOv8 model
model = YOLO("best_3.pt")  # Ensure you have a trained model

# Open the webcam or video file
cap = cv2.VideoCapture(0)  # Change to a file path if using an image or video


def get_angle_and_draw_line(binary_img, x1, y1, x2, y2, frame, is_vertical):
    # Get non-zero points inside the box
    points = cv2.findNonZero(binary_img)

    if points is not None and len(points) > 1:
        # Find the topmost and bottommost or leftmost and rightmost points
        if is_vertical:
            # Sort by y-axis (top and bottom points)
            points = sorted(points, key=lambda pt: pt[0][1])
            point1, point2 = tuple(points[0][0]), tuple(points[-1][0])
        else:
            # Sort by x-axis (left and right points)
            points = sorted(points, key=lambda pt: pt[0][0])
            point1, point2 = tuple(points[0][0]), tuple(points[-1][0])

        # Offset points to be inside the box
        point1_offset = (point1[0] + x1, point1[1] + y1)
        point2_offset = (point2[0] + x1, point2[1] + y1)

        # Draw line inside the box
        cv2.line(frame, point1_offset, point2_offset, (0, 0, 255), 2)

        # Calculate angle relative to north (90 degrees for vertical)
        delta_x, delta_y = point2[0] - point1[0], point2[1] - point1[1]
        angle_rad = math.atan2(delta_y, delta_x)
        angle_deg = (angle_rad * 180.0 / np.pi) % 360

        # Adjust angle for vertical/horizontal
        if is_vertical:
            angle_deg = (angle_deg + 90) % 360

        # Display angle near the box
        angle_text = f"Angle: {angle_deg:.2f}°"
        cv2.putText(frame, angle_text, (x1, y1 - 20), cv2.FONT_ITALIC, 0.5, (0, 255, 255), 2)


# Main loop
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLOv8 inference on the frame
    results = model(frame)

    # Draw bounding boxes on the detected resistors
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
            confidence = box.conf[0].item()

            if confidence > 0.5:
                # Crop the region inside the bounding box
                roi = frame[y1:y2, x1:x2]

                # Convert to grayscale and apply binary threshold to isolate content
                gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                _, binary_roi = cv2.threshold(gray_roi, 50, 255, cv2.THRESH_BINARY_INV)

                # Determine if the box is taller or wider
                width, height = x2 - x1, y2 - y1
                is_vertical = height > width

                # Get angle and draw line inside the box
                get_angle_and_draw_line(binary_roi, x1, y1, x2, y2, frame, is_vertical)

                # Draw the bounding box
                label = f"Resistor {confidence:.2f}, x:{(x1 + x2) / 2:.1f} y:{(y1 + y2) / 2:.1f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_ITALIC, 0.5, (0, 255, 0), 2)

    # Display the result
    cv2.imshow("Detection with Angle", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
