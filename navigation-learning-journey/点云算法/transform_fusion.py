#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion
import tf
from collections import deque
import math

# 定义移动平均滤波器窗口大小
MA_WINDOW_SIZE = 2  # 例如，取2个最近的变换数据进行平均

# 初始化全局变量以存储过去的变换数据
translation_queue = deque(maxlen=MA_WINDOW_SIZE)
rotation_queue = deque(maxlen=MA_WINDOW_SIZE)

# 初始化全局变量以存储上一次的平移和旋转
last_translation = (0.0, 0.0, 0.0)
last_rotation = (0.0, 0.0, 0.0, 1.0)

def transform_callback(msg):
    global last_translation, last_rotation
    global translation_queue, rotation_queue
    br = tf.TransformBroadcaster()
    
    # 将当前变换数据添加到队列中
    translation_queue.append((msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z))
    rotation_queue.append((msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w))
    
    # 检查队列是否已满
    if len(translation_queue) == MA_WINDOW_SIZE:
        # 计算平移的移动平均值
        sum_x = sum(t[0] for t in translation_queue)
        sum_y = sum(t[1] for t in translation_queue)
        sum_z = sum(t[2] for t in translation_queue)
        mean_translation = (sum_x / MA_WINDOW_SIZE, sum_y / MA_WINDOW_SIZE, sum_z / MA_WINDOW_SIZE)
        
        # 计算旋转的移动平均值（这里简化处理，直接取最后一个旋转值）
        mean_rotation = rotation_queue[-1]
        
        
        # 发布滤波后的变换
        br.sendTransform(
            (mean_translation[0], mean_translation[1], mean_translation[2]),
            (mean_rotation[0], mean_rotation[1], mean_rotation[2], mean_rotation[3]),
            msg.header.stamp, 
            msg.child_frame_id, 
            msg.header.frame_id
        )
        
        rospy.loginfo("Published smoothed transform with x: %f, y: %f", mean_translation[0], mean_translation[1])
    else:
        rospy.loginfo("Accumulating data for moving average filter")

def listener_node():
    rospy.init_node('tf_listener_node', anonymous=True)
    rospy.Subscriber("map_to_odom", TransformStamped, transform_callback)
    rospy.spin()

if __name__ == '__main__':
    listener_node()