#!/usr/bin/env python2

import rospy, serial, time
from yolov3_trt_ros.msg import IDistanceList, ObjectID

object_id_msg = ObjectID()
id_distance = {}
obj_id = -1
detect_distance = 110

def callback(data) :
    global id_distance
    global obj_id
    
    distance_list = []
    distance_list.extend(data.distance_list)
    id_list = []
    id_list.extend(data.id_list)
    
    id_distance = {id_list[i]:distance_list[i] for i in range(0, len(distance_list))}     
    id_distance = sorted(id_distance.items(), key=lambda x: x[1])
    
    if id_distance[0][1] <= detect_distance:
        obj_id = id_distance[0][0]

def drive_left():
    global object_id_msg
    object_id_msg.object_id = obj_id
    pub.publish(object_id_msg)

def drive_right():
    global object_id_msg
    object_id_msg.object_id = obj_id
    pub.publish(object_id_msg)

def drive_stop():
    global object_id_msg
    object_id_msg.object_id = obj_id
    pub.publish(object_id_msg)

def find_traffic_green():
    global object_id_msg
    object_id_msg.object_id = obj_id
    pub.publish(object_id_msg)
    
def find_traffic_yellow():
    global object_id_msg
    object_id_msg.object_id = obj_id
    pub.publish(object_id_msg)
    
def find_traffic_red():
    global object_id_msg
    object_id_msg.object_id = obj_id
    pub.publish(object_id_msg)

def find_cross_walk():
    print 'cross walk'
    global object_id_msg
    object_id_msg.object_id = obj_id
    pub.publish(object_id_msg)
    
def find_car():
    global object_id_msg
    object_id_msg.object_id = obj_id
    return

rospy.init_node('trt_driver')

rospy.Subscriber('/idistance_list', IDistanceList, callback, queue_size=1)
pub = rospy.Publisher('/object_detect',ObjectID,queue_size=1)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if obj_id == 0:
        find_cross_walk()
    elif obj_id == 2:
        drive_left()
    elif obj_id == 3:
        drive_right()
    elif obj_id == 4:
        drive_stop()
    elif obj_id == 5:
        find_car()
    elif obj_id == 6:
        find_traffic_green()
    elif obj_id == 7:
        find_traffic_yellow()
    elif obj_id == 8:
        find_traffic_red()
    
    #reset obj_id
    obj_id = -1

    rate.sleep()