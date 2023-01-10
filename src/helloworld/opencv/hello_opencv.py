import cv2

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
    cv2.circle(frame, (width // 2, height // 2), 50, (0, 0, 255), 3)
    # Draw diagonal lines from top-left to bottom-right and top-right to bottom-left
    cv2.line(frame, (0, 0), (width, height), (0, 255, 0), 3)
    cv2.line(frame, (width, 0), (0, height), (0, 255, 0), 3)
    # Draw a text on the frame
    cv2.putText(frame, 'Press q to quit!', (width//2 - 100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

def show_capture(capture_window_name, capture):
    while True:
        # Capture frame-by-frame
        ret, frame = capture.read()
        overlaid_image = draw_overlay(frame)
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
    show_capture(capture_window_name, capture)
    cleanup_capture(capture)

if __name__ == '__main__':
    main()