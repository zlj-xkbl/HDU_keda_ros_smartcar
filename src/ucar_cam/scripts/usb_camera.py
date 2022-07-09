#! /usr/bin/python2.7
# coding=utf-8
import numpy as np
import cv2
import time
import rospy

cap = cv2.VideoCapture("/dev/video0")
weight=1920
height=1080
cap.set(4, height)
codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
print(codec)

cap.set(cv2.CAP_PROP_FOURCC, codec)
fps =cap.get(cv2.CAP_PROP_FPS) #获取视频帧数
# cap.set(cv2.CAP_PROP_AUTOFOCUS, False)  # 禁止自动对焦
# cap.set(cv2.CAP_PROP_SETTINGS, 1) 
b_fps=time.time()  #后帧时间全局变量赋值
while(True):
    # 读取一帧
    f_fps=time.time() #前帧时间
    fps_now=str(round(1/(f_fps-b_fps),2))   #求当前帧率
    b_fps=f_fps #后帧时间
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)   ##图像左右颠倒
    cv2.putText(frame,'FPS:'+' '+fps_now,(10, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1 ,(0,0,255),2,cv2.LINE_AA)
    h,w=frame.shape[:2]
    print(h,w)
    print("获得的帧率:",fps)
    cv2.imshow('Camera_USB', frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break
cap.release()
cv2.destroyAllWindows()
