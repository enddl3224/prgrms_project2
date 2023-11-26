#!/usr/bin/env python

import cv2
import message_filters
import numpy as np
from pyquaternion import Quaternion
import rospy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import yaml
import math
from Calibrator import Calibrator
from Homography import Homography

from yolov3_trt_ros.msg import BoundingBox, BoundingBoxes, IDistanceList
from data_processing import ALL_CATEGORIES


map1 = np.empty(shape=[0])
map2 = np.empty(shape=[0])

class Reprojection():
    def __init__(self):
        global map1, map2
        
        self.debug_c = False
        self.pub_bev = False
        
        rospy.init_node('reprojection')

        self.calibrator = Calibrator()
        map1, map2 = self.calibrator.get_map()
        
        self.homography = Homography()
        self.lp = lg.LaserProjection()
        self.set_param_in_launch()

        self.pub = rospy.Publisher('/idistance_list', IDistanceList, queue_size=1)
        self.bev_pub = rospy.Publisher('/bev_image', Image, queue_size=1)
        
        self.data_sub = rospy.Subscriber('/yolov3_trt_ros/detections', BoundingBoxes, self.bbox_callback, queue_size=1)
        self.lidar_sub = message_filters.Subscriber('/scan', LaserScan, queue_size=1)
        self.image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image, queue_size=1)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.image_sub], 1, 0.3)
        self.ts.registerCallback(self.sensor_callback)
        
        self.get_KD()
        self.set_tvec_rvec()
        
        self.boxes = list()
        
        self.run()

    def set_param_in_launch(self):
        self.scan_topic = rospy.get_param("~scan_topic")
        self.image_topic = rospy.get_param("~image_topic")
        self.calib_file = rospy.get_param("~calib_file")
        self.config_file = rospy.get_param("~config_file")
        self.laser_point_radius = rospy.get_param("~laser_point_radius")
    
    def set_tvec_rvec(self):
        with open(self.calib_file, 'r') as f:
            data = f.read().split()
            qx = float(data[0])
            qy = float(data[1])
            qz = float(data[2])
            qw = float(data[3])
            tx = float(data[4])
            ty = float(data[5])
            tz = float(data[6])
            
        self.q = Quaternion(qw,qx,qy,qz).transformation_matrix
        
        self.q[0,3] = tx
        self.q[1,3] = ty
        self.q[2,3] = tz
        
        print("Extrinsic parameter - camera to laser")
        print(self.q)
        
        self.tvec = self.q[:3,3]
        rot_mat = self.q[:3,:3]
        self.rvec, _ = cv2.Rodrigues(rot_mat)

    def get_KD(self):
        with open(self.config_file, 'r') as f:
            f.readline()
            config = yaml.load(f)
            
        self.lens = config['lens']
        fx = float(config['fx'])
        fy = float(config['fy'])
        cx = float(config['cx'])
        cy = float(config['cy'])
        k1 = float(config['k1'])
        k2 = float(config['k2'])
        p1 = float(config['p1/k3'])
        p2 = float(config['p2/k4'])
          
        self.K = np.matrix([[fx, 0.0, cx],
                            [0.0, fy, cy],
                            [0.0, 0.0, 1.0]])
        self.D = np.array([k1, k2, p1, p2])
        
        print("Camera parameters")
        print("Lens = %s" % self.lens)
        print("K =")
        print(self.K)
        print("D =")
        print(self.D)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
    
    def bbox_callback(self, bboxes):
        self.boxes = bboxes.bounding_boxes
        
    def sensor_callback(self, scan, image):
        image = np.frombuffer(image.data, dtype = np.uint8).reshape(image.height, image.width, -1)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
        
        cloud = self.lp.projectLaser(scan)
        points_for_obj = pc2.read_points(cloud)
        points_for_dist = pc2.read_points(cloud, field_names = ("x", "y", "z"))
        
        lidar_distance_list = [map(get_norm2, list(points_for_dist))][0]
        
        objPoints = np.array(map(extract, points_for_obj))
        Z = get_z(self.q, objPoints, self.K)
        
        objPoints = objPoints[Z > 0]
        lidar_distance_list = [x for i, x in enumerate(lidar_distance_list) if Z[i] > 0]
        
        if self.lens == 'pinhole':
            lidar2img_points, _ = cv2.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        elif self.lens == 'fisheye':
            objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
            lidar2img_points, _ = cv2.fisheye.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
          
        lidar2img_points = np.squeeze(lidar2img_points)

        image, distance_list, id_list = self.draw_infomation(self.boxes, image, lidar2img_points, lidar_distance_list)

        if self.debug_c:
            cv2.imshow('image', image)
            cv2.waitKey(1)
        
        if distance_list is not None and id_list is not None:
            self.id_distance_publish(distance_list, id_list)

        
    def id_distance_publish(self, distance_list, id_list):
        res = IDistanceList()
        res.distance_list = distance_list
        res.id_list = id_list
        self.pub.publish(res)
        
    def bev_image_publish(self, image):
        image_msg = Image()
        image_msg.header.stamp = rospy.Time.now()
        image_msg.width = 270
        image_msg.height = 270
        image_msg.data = image.flatten()
        self.bev_pub.publish(image_msg)

    def draw_infomation(self, boxes, image, lidar2img_points, lidar_distance_list):
        all_categories=ALL_CATEGORIES
        for lidar_point in lidar2img_points:
            try:
                cv2.circle(image, (int(lidar_point[0]),int(lidar_point[1])), 2, (0,255,0), 2)           
            except OverflowError:
                continue
        
        if self.pub_bev:
            bev_image = np.ndarray(shape=(270,270,3), dtype=np.uint8)
            bev_image.fill(255)
            cv2.line(bev_image, (0, 0), (0, 270), (128, 128, 128))
            cv2.line(bev_image, (45, 0),(45, 270), (128, 128, 128))
            cv2.line(bev_image, (90, 0), (90, 270), (128, 128, 128))
            cv2.line(bev_image, (135, 0), (135, 270), (128, 128, 128))
            cv2.line(bev_image, (180, 0), (180, 270), (128, 128, 128))
            cv2.line(bev_image, (225, 0), (225, 270), (128, 128, 128))
            cv2.line(bev_image, (270, 0), (270, 270), (128, 128, 128))

            cv2.line(bev_image, (0, 0), (270, 0), (128, 128, 128))
            cv2.line(bev_image, (0, 45),(270, 45), (128, 128, 128))
            cv2.line(bev_image, (0, 90), (270, 90), (128, 128, 128))
            cv2.line(bev_image, (0, 135), (270, 135), (128, 128, 128))
            cv2.line(bev_image, (0, 180), (270, 180), (128, 128, 128))
            cv2.line(bev_image, (0, 225), (270, 225), (128, 128, 128))
            cv2.line(bev_image, (0, 270), (270, 270), (128, 128, 128))

        if len(boxes) == 0:
            if self.bev_pub: self.bev_image_publish(bev_image)
            return image, None, None
        
        homo_distance_list = self.homography.get_distance(image, boxes)
        
        distance_list = []
        id_list = []
        
        
        for box, homo_distance in zip(boxes, homo_distance_list):
            prob = box.probability
            x_min = box.xmin
            y_min = box.ymin
            x_max = box.xmax
            y_max = box.ymax
            class_id = box.id
            
            print class_id
            x_min = x_min * 640 / 416
            x_max = x_max * 640 / 416
            y_min = y_min * 480 / 416
            y_max = y_max * 480 / 416
            
            x_min = int(x_min)
            x_max = int(x_max)
            y_min = int(y_min)
            y_max = int(y_max)

            lidar_list = [lidar_distance_list[i] for i, x in enumerate(lidar2img_points) if x[0] >= x_min if x[1] <= x_max]
            
            distance = None
            
            if lidar_list is None:
                distance = min(lidar_list) * 100
            
            if distance is None:
                distance = homo_distance
            
            distance_list.append(distance)
            id_list.append(class_id) 
            
            if self.debug_c:
                cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)          
                cv2.putText(image, '{0} {1:.2f} {2}'.format(all_categories[class_id], prob, int(distance)), (x_min, y_min - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
            
            if self.pub_bev:    
        
                try:
                    radian = math.atan2(135 - ((x_min + x_max) / 2.0), 270 - ((y_min + y_max) / 2.0))
                    x_val = distance * math.cos(radian * math.pi / 180)
                    y_val = distance * math.sin(radian * math.pi / 180)
                    
                    if x_val <= 0: x_coord = 135 - x_val
                    else: x_coord = 135 + x_val
                    y_coord = 270 - y_val 
                    
                    if x_coord is not None and y_coord is not None:
                        cv2.circle(bev_image, (int(x_coord), int(y_coord)), 1, (0,0,0), 1)
                        cv2.putText(bev_image, '{0} {1}'.format(all_categories[class_id], int(distance)), (int(x_coord),int(y_coord) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))           
                except OverflowError:
                    continue
                    
                cv2.imshow('bev_image', bev_image)
                cv2.waitKey(1)
                    
        if self.pub_bev:
           self.bev_image_publish(bev_image)
                        
        return image, distance_list, id_list

def get_z(T_cam_world, T_world_pc, K):
    R = T_cam_world[:3,:3]
    t = T_cam_world[:3,3]
    proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
    xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
    xy_hom = np.dot(proj_mat, xyz_hom.T).T
    z = xy_hom[:, -1]
    z = np.asarray(z).squeeze()
    return z

def extract(point):
    return [point[0], point[1], point[2]]

def get_norm2(point):
    return ((point[0] ** 2) + (point[1] ** 2) + (point[2] ** 2)) ** 0.5


if __name__ == '__main__':
    reprojection = Reprojection()
    rospy.init_node('reprojection', anonymous=False)
    reprojection.run()