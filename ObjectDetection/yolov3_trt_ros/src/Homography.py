import cv2
import numpy as np

class Homography:
    camera_matrix = np.array([[349.475340021459, 0.000000, 327.7321913497484],
                    [0.000000, 351.21490491019074, 211.1069552491865],
                    [0.000000, 0.000000, 1.000000]])
    
    image_points = np.array([
        [205, 415],
        [314, 415],
        [205, 257],
        [240, 258],
        [205, 228],
        [226, 229],
        [205, 216],
        [220, 216]
    ], dtype=np.float32)

    object_points = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 10.0, 0.0],
        [0.0, 0.0, 45.0],
        [0.0, 10.0, 45.0],
        [0.0, 0.0, 90.0],
        [0.0, 10.0, 90.0],
        [0.0, 0.0, 135.0],
        [0.0, 10.0, 135.0]
    ], dtype=np.float32)
    
    DATA_SIZE = 8

    image = np.empty(shape=[0])
    
    homo_object_point = np.empty(shape=[0])
    
    homography = np.empty(shape=[0])
    
    def __init__(self):
        self.homo_object_point = np.append(self.object_points[:,2:3], -self.object_points[:,1:2], axis=1)
        self.homo_object_point = np.append(self.homo_object_point, np.ones([1, self.DATA_SIZE]).T, axis=1)
        self.homography, _ = cv2.findHomography(self.image_points, self.homo_object_point)
        
    def get_distance(self, image, bboxes):
        
        if bboxes is None:
            return image
        
        self.image = image
        
        distance_list = []
        
        for bbox in bboxes:
            x_min = bbox.xmin
            x_max = bbox.xmax
            y_min = bbox.ymin
            y_max = bbox.ymax
            
            image_point = np.array([(x_min + x_max)/2, y_max, 1])
            estimation_distance = np.dot(self.homography, image_point)

            x = estimation_distance[0]
            y = estimation_distance[1]
            z = estimation_distance[2]

            x = x/z
            y = y/z
            z = z/z

            distance = ((x ** 2 + y ** 2 + z ** 2) ** 0.5)
            distance_list.append(distance)
            # use astype(int)
            
            # cv2.circle(image, (int((left+right)/2), int(bottom)), 2, (255,255,0))
            # cv2.putText(image, "distance : "+str(round(distance,2)) + "cm",(int((left+right)/2), int(bottom)), 2, 0.5, (10,250,10),1)
        # return image
        return distance_list
        