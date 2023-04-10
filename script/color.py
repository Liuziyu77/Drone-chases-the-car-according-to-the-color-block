#!/usr/bin/env python
#-*- coding: utf-8 -*-
"""
look here:
————————————————>  x轴和w的正向,范围在0-1280
|
|
|
|
|
↓
y轴和h的正向,范围在0-720

本程序计算色块中心位置与相机视野中心位置的偏移量,并以自定义的消息类型bias-msg发布


"""
from __future__ import print_function
 
 
import sys
import rospy
import time
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Vector3
from cam.msg import bias
 
class image_converter:
 
  def __init__(self):
    # 定义处理前图像订阅器
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    # 定义处理后图像发布器
    self.image_pub = rospy.Publisher("/image_topic_2",Image, queue_size =3)
    # 定义色块中心位置距离相机视野中心点偏移量发布器
    self.bias_pub = rospy.Publisher("/color_block_bias",bias, queue_size =10)
    # 定义原图和opencv图转换器
    self.bridge = CvBridge()

 
  def callback(self,data):
    # 将原图转为opencv图
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print ('image_msg to img_opencv transform failed')
    # 选取色块的取值范围
    color_r_lower=np.array([0,100,100])
    color_r_upper = np.array([10, 255, 255])
    # color_b_lower=np.array([110,100,100])
    # color_b_upper = np.array([130,255,255])
    color_b_lower=np.array([100,100,70])
    color_b_upper = np.array([135,255,255])
    
    # 捕捉蓝色色块区域
    frame_=cv2.GaussianBlur(frame,(5,5),0)                    
    hsv=cv2.cvtColor(frame_,cv2.COLOR_BGR2HSV)
    mask_b=cv2.inRange(hsv,color_b_lower,color_b_upper)   
    mask_b=cv2.erode(mask_b,None,iterations=2)
    mask_b=cv2.dilate(mask_b,None,iterations=2)
    mask_b=cv2.GaussianBlur(mask_b,(3,3),0)     
    countours_b,hierarchy=cv2.findContours(mask_b.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    # 捕捉红色色块区域
    # frame_=cv2.GaussianBlur(frame,(5,5),0)                    
    # hsv=cv2.cvtColor(frame_,cv2.COLOR_BGR2HSV)
    # mask_r=cv2.inRange(hsv,color_r_lower,color_r_upper)   
    # mask_r=cv2.erode(mask_r,None,iterations=2)
    # mask_r=cv2.dilate(mask_r,None,iterations=2)
    # mask_r=cv2.GaussianBlur(mask_r,(3,3),0)     
    # countours_r,hierarchy=cv2.findContours(mask_r.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    # 如果存在蓝色色块，则进行一下操作
    if not(len(countours_b)==0):
        cv2.drawContours(frame,countours_b,-1,(255,255,255),3)
        # cv2.drawContours(frame,countours_r,-1,(0,0,0),3)
        # 将opencv图转为原图并发布
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
          print('img_opencv to img_msg transform failed')
        # 找到最大的色块    
        cnt = max (countours_b,key=cv2.contourArea)
        # 获取x,y,w,h，分别是最大色块矩形框的左上角坐标，宽，高
        (color_x,color_y,color_w,color_h)=cv2.boundingRect(cnt)
        # 绘制矩形框
        if color_w > 20 and color_h > 20:
          cv2.rectangle(frame,(int(color_x),int(color_y)),(int(color_x+color_w),int(color_y+color_h)),(255,0,255),3) 
          # print(color_x,color_y,color_w,color_h)
          # 计算色块中心到相机视野中心的偏移量（相机像素1280×720）
          color_center_x = color_x + 0.5*color_w
          color_center_y = color_y + 0.5*color_h
          bias_x = 640 - color_center_x
          bias_y = 360 - color_center_y
          # 创建自定义的bias消息类型，并发布
          bias_msg = bias()
          bias_msg.bias_x = int(bias_x)
          bias_msg.bias_y = int(bias_y)
          bias_msg.flage = 1
          self.bias_pub.publish(bias_msg)
          print(bias_msg.bias_x,bias_msg.bias_y)
        # 显示
        cv2.imshow("Image window", frame)
        cv2.waitKey(3)
    else:
        # 创建自定义的bias消息类型，并发布
        bias_msg = bias()
        bias_msg.bias_x = 0
        bias_msg.bias_y = 0
        bias_msg.flage = 0
        self.bias_pub.publish(bias_msg)
        print('NO COLOR BLOCK IS DETECTED',bias_msg.bias_x,bias_msg.bias_y)
        # 显示
        cv2.imshow("Image window", frame)
        cv2.waitKey(3)
           
    
    
if __name__ == '__main__':
  rospy.init_node('position_bias_compute', anonymous=True)
  ic = image_converter()
  print('start')
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print('exception')
