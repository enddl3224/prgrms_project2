import cv2
import numpy as np

class Calibrator:
        
    calibrate_mtx = np.array([
        [349.475340021459, 0.000000, 327.7321913497484,],
        [0.000000, 351.21490491019074, 211.1069552491865,],
        [0.000000, 0.000000, 1.000000]
    ], dtype=np.float64)
    
    dist = np.array([-0.05339752213986486, 0.019285760147283504, -0.0759693139820371, 0.06693911354667991], dtype=np.float64)
    
    def __init__(self):
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.calibrate_mtx, self.dist, np.eye(3), self.calibrate_mtx, (640,480), cv2.CV_32FC1)
    
    def get_map(self):
        return self.map1, self.map2

    def get_KD(self):
        return self.calibrate_mtx, self.dist