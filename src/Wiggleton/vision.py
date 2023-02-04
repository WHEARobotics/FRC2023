from cscore import CameraServer
import numpy as np 

def main():
#     print ("Vision started")
    cs = CameraServer.getInstance()
    camera = cs.startAutomaticCapture()
    camera = cs.addAxisCamera()
    camera.setResolution(320, 240)
    cvSink = cs.getVideo()
    outputStream = cs.putVideo("Rectangle", 640, 480)
    img = np.zeros(shape=(640,480,3), dtype=np.uint8)
    while True:
        time, img = cvSink.grabFrame(img)
        # If there is an error, send it to output stream
        if time == 0:
            outputStream.notifyError(cvSink.getError())
            continue

        cv2.rectangle(img, (100, 100), (300,300), (255,0,0), 5)
        outputStream.putFrame(img)


