#!/usr/bin/env python3
# Straight-line trim test: drive open loop on /cmd_vel and report the yaw change from /odom_rf2o.
# Nav2 must be idle (no goal).  Check >0.5 m clearance front and back first.
#   python3 trim_test.py 0.14 0 0 0.7     # forward 0.7 s
#   python3 trim_test.py -0.14 0 0 0.7    # reverse 0.7 s
# dyaw < 0 -> drifts right -> raise STRAIGHT_TRIM_FORWARD/REVERSE; dyaw > 0 -> lower it.
import rclpy, math, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
vx,vy,wz,dur=map(float,sys.argv[1:5])
rclpy.init(); n=Node('pulse2'); pub=n.create_publisher(Twist,'/cmd_vel',1); odo=[]
def oc(m):
    q=m.pose.pose.orientation
    odo.append((m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
n.create_subscription(Odometry,'/odom_rf2o',oc,50)
t=time.time()
while not odo and time.time()-t<5: rclpy.spin_once(n,timeout_sec=0.1)
t=time.time()
while time.time()-t<1.0: rclpy.spin_once(n,timeout_sec=0.02)
n0=len(odo); p0=odo[-1]; m=Twist(); m.linear.x=vx; m.linear.y=vy; m.angular.z=wz
t0=time.time()
while time.time()-t0<dur: pub.publish(m); rclpy.spin_once(n,timeout_sec=0.02)
ts=time.time()
while time.time()-ts<1.5: pub.publish(Twist()); rclpy.spin_once(n,timeout_sec=0.02)
p=odo[-1]; c,s=math.cos(p0[2]),math.sin(p0[2]); dx,dy=p[0]-p0[0],p[1]-p0[1]
fwd,lat=dx*c+dy*s,-dx*s+dy*c; dyaw=math.atan2(math.sin(p[2]-p0[2]),math.cos(p[2]-p0[2]))
print('[odom msgs %d] '%(len(odo)-n0),end=''); print('vx=%.2f wz=%.2f T=%.2f | fwd=%.3f lat=%.3f dyaw=%.3f rad | yaw/m=%.2f'%(vx,wz,dur,fwd,lat,dyaw,dyaw/fwd if abs(fwd)>0.02 else 0))
