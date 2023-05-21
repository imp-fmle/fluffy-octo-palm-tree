#!/usr/bin/env python3

import rospy
from pr_5.msg import Robot_systems
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def talker():

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    pub = rospy.Publisher('custom_chatter', Robot_systems)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10 hz

    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w' :
            control_linear_vel  += 0.1
        elif key == 'x' :
            control_linear_vel  -= 0.1
        elif key == 'a' :
            control_angular_vel  -= 0.1
        elif key == 'd' :
            control_angular_vel  += 0.1
        elif key == ' ' or key == 's' :
            control_linear_vel  = 0.0
            control_angular_vel = 0.0
        else:
            if (key == '\x03'):
                break

        msg = Robot_systems()
        msg.linear_vel = control_linear_vel
        msg.angular_vel = control_angular_vel
        msg.time = rospy.get_time()

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        talker()
    except rospy.ROSInterruptException: pass
