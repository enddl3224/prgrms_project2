import cv2
import numpy as np

image_width, image_height = 640, 480

capture = cv2.VideoCapture(3)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

calibrate_mtx = np.array([
[349.475340021459, 0.000000, 327.7321913497484,],
[0.000000, 351.21490491019074, 211.1069552491865,],
[0.000000, 0.000000, 1.000000]
])
dist = np.array([-0.05339752213986486, 0.019285760147283504, -0.0759693139820371, 0.06693911354667991])

map1, map2 = cv2.fisheye.initUndistortRectifyMap(calibrate_mtx, dist, np.eye(3), calibrate_mtx, (640,480), cv2.CV_32FC1)

def calibrate_image(src):
    global image_width, image_height

    calibrated_image = cv2.remap(src, map1, map2, cv2.INTER_LINEAR)

    return cv2.resize(calibrated_image, (image_width, image_height))

i = 0
idx = 0

while True:
    _, src = capture.read()
    cal = calibrate_image(src)
    

    cv2.imshow("calibrated image 1", cal)
    if i % 30 == 0:
        cv2.imwrite('./output/'+str(idx) + '.jpg', cal)
    
        idx += 1

    i += 1

    if cv2.waitKey(10) == 27:
        cv2.capture.release()
        cv2.destoryAllWindow()
        break
