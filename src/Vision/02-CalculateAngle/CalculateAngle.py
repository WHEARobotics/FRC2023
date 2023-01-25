import cv2
import robotpy_apriltag
from wpimath.geometry import Transform3d

import math
import pytest


def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    # FRC 2023 uses tag16h5 (game manual 5.9.2)
    assert detector.addFamily("tag16h5")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(
    robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.2, 500, 500, frame_size[1] / 2.0, frame_size[0] / 2.0
        )
    )
    return detector, estimator

def get_capture(window_name, video_capture_device_index=0):
    # Create a window named 'window_name'
    cv2.namedWindow(window_name)
    # Open the Webcam
    cap = cv2.VideoCapture(video_capture_device_index)
    return cap

def draw_overlay(frame):
    # Get the height and width of the frame
    height, width, channels = frame.shape
    # Draw a circle in the center of the frame
    cv2.circle(frame, (width // 2, height // 2), 50, (0, 0, 255), 1)
    # Draw diagonal lines from top-left to bottom-right and top-right to bottom-left
    cv2.line(frame, (0, 0), (width, height), (0, 255, 0), 1)
    cv2.line(frame, (width, 0), (0, height), (0, 255, 0), 1)
    # Draw a text on the frame
    cv2.putText(frame, 'q to quit', (width//2 - 100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

def process_apriltag(estimator, tag):
    tag_id = tag.getId()
    center = tag.getCenter()
    hamming = tag.getHamming()
    decision_margin = tag.getDecisionMargin()
    print("Hamming for {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))

    est = estimator.estimateOrthogonalIteration(tag, 50)
    return tag_id, est.pose1, center

def draw_tag(frame, result):
    assert frame is not None
    assert result is not None
    tag_id, pose, center = result
    print(center)
    cv2.circle(frame, (int(center.x), int(center.y)), 50, (255, 0, 255), 3)
    msg = f"Tag ID: {tag_id} Pose: {pose}"
    cv2.putText(frame, msg, (50, 400 + int(tag_id * 50)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

def draw_angle(frame, angle_x, radians_per_pixel):
    assert frame is not None
    # Get the height and width of the frame
    height, width, channels = frame.shape
    # Draw a line angle_x radians from the center of the frame
    offset_x = angle_x / radians_per_pixel
    x = int(width // 2 + offset_x)
    cv2.line(frame, (x, 0), (x, height), (255, 0, 255), 1)
    # Draw a text on the frame
    msg = f"Angle: {angle_x:0.3f} rad"
    msg_loc = (int(width // 2 + offset_x - 100), 250)
    cv2.putText(frame, msg, msg_loc, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
    return frame

def calculate_x_angle(radians_per_pixel, frame, tag_center_x):
    frame_center_x = frame.shape[1] // 2

    delta_x = tag_center_x - frame_center_x

    angle_x = delta_x * radians_per_pixel
    return angle_x

def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Change this value to the value you calculated in the "Webcam calibration" exercise
    RADIANS_PER_X_PIXEL = 5.75625e-04
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    DETECTION_MARGIN_THRESHOLD = 100
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    # Note that results will be empty if no apriltag is detected
    for result in results:
        frame = draw_tag(frame, result)
        apriltag_center = result[2]
        tag_center_x = apriltag_center.x
        angle_x = calculate_x_angle(RADIANS_PER_X_PIXEL, frame, tag_center_x)
        frame = draw_angle(frame, angle_x, RADIANS_PER_X_PIXEL)
    return frame

def show_capture(capture_window_name, capture, detector, estimator):
    while True:
        # Capture frame-by-frame
        ret, frame = capture.read()
        # Detect apriltag
        frame_with_maybe_apriltags = detect_and_process_apriltag(frame, detector, estimator)

        overlaid_image = draw_overlay(frame_with_maybe_apriltags)
        # Display the resulting frame in the named window
        cv2.imshow(capture_window_name, overlaid_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def cleanup_capture(capture):
    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()

def main():
    capture_window_name = 'Capture Window'
    capture = get_capture(capture_window_name, 0)
    detector, estimator = get_apriltag_detector_and_estimator((1080,1920))
    show_capture(capture_window_name, capture, detector, estimator)
    cleanup_capture(capture)

if __name__ == '__main__':
    main()
