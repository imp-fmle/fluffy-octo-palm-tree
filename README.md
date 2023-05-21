# fluffy-octo-palm-tree

cd ~/catkin_ws
source ./devel/setup.bash

#1
roscore
#2
rosrun pr_5 talker_custom.py
#3
rosrun pr_5 listener_custom.py
#4
roslaunch pr_5 rviz.launch

## msg_file
```python
string name
float32 time
float32 linear_vel
float32 angular_vel
```

# Talker

<details>
  <summary>Toggle contents of talker_custom.py </summary>
  
  ```python
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
  ```
</details>

  
# Listener

<details>
  <summary>Toggle contents of listener_custom.py </summary>
  
  ```python
  #!/usr/bin/env python

  import rospy
  import math
  from pr_5.msg import Robot_systems

  encr = 0
  encl = 0
  w_lg = 0
  w_rg = 0
  teta_g = 0
  x_g = 0
  y_g = 0
  t_g = 0

  def wspeed(w_l, w_r):
      global t_g
      N = 4096
      t = float(rospy.get_time())
      dt = t - t_g
      #вычисление показаний энкодера
      dencl = int(w_l * dt * N / (2 * 3.1415))
      dencr = int(w_r * dt * N / (2 * 3.1415))
      #вычисление угловой скорости колеса
      wl = dencl * 2 * 3.1415 / dt / N
      wr = dencr * 2 * 3.1415 / dt / N
      t_g = t
      return dt, wl,wr,  dencl, dencr
  def funny(V, W):
      global t_g
      global teta_g
      global x_g
      global y_g
      global t_g
      global encr
      global encl
      global w_rg
      global w_lg
      L = 0.287
      r = 0.033
      #угловые скорости колёс по полученным данным
      wl = (V - 0.5 * L * W) / r
      wr = (V + 0.5 * L * W) / r
      #угловые скорости колёс после сау
      dt, wl,wr, dencl, dencr = wspeed(wl, wr)
      encl += dencl
      encr += dencr
      #посчитанные угловая и линейная скорости
      V = 0.5 * r * (wl + wr)
      W = r * (wr - wl) / L
      teta = W * dt
      x = V * math.cos(teta) * dt
      y = V * math.sin(teta) * dt
      x_g += x
      y_g += y
      teta_g += teta
      return t_g

  def callback(data):
      funny(data.linear_vel, data.angular_vel)
      rospy.loginfo("Encoder_left %0.2f", encl)
      rospy.loginfo("Encoder_right %0.2f", encr)
      rospy.loginfo("Y: %0.2f", y_g)
      rospy.loginfo("X: %0.2f", x_g)

  def listener():
      rospy.init_node('custom_listener', anonymous=True)
      rospy.Subscriber("custom_chatter", Robot_systems, callback)
      t_g = float(rospy.get_time())
      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()

  if __name__ == '__main__':
      listener()
  ```
</details>

![image](./screenshots/Screenshot 2023-05-21 at 20.57.34.png)
