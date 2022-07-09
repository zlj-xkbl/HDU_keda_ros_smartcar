#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from xf_mic_asr_offline.srv import Set_Awake_Word_srv
# class set_awake_word():
#     def __init__(self):
#         self.set_awake_word()

#     def set_awake_word(self):
#         rospy.wait_for_service('/xf_asr_offline_node/set_awake_word_srv')#发布语音合成服务
#         mic_data = rospy.ServiceProxy('/xf_asr_offline_node/set_awake_word_srv', Set_Awake_Word_srv)
#         response = mic_data("小车启动")
def main():
    rospy.init_node('xf_set_awake_word_srv')
    rospy.wait_for_service('/xf_asr_offline_node/set_awake_word_srv')#发布语音合成服务
    try:
        mic_data = rospy.ServiceProxy('/xf_asr_offline_node/set_awake_word_srv', Set_Awake_Word_srv)
        response = mic_data("小车启动")
        print(response)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
