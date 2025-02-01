#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Twist


move_cmd = Twist()

def callback(message: Twist):
    global move_cmd
    move_cmd = message
   
    


    

def main():
    global move_cmd
    topic1 = '/cmd_vel';
    topic2= '/jb_0/cmd_vel';
        
    rospy.init_node('remap_node',anonymous=True);
   
    rospy.Subscriber(topic1,Twist, callback);
    pub = rospy.Publisher(topic2, Twist,queue_size=10);
    my_check = True

    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        
            


   



if __name__ == '__main__':

    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber is error');
        pass;
