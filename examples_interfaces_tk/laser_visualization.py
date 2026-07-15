import rospy
from jackal_msgs.msg import Drive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.spatial.transform import Rotation as R
import math, time
import os, sys
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from lab_utils.ip_config import get_ip
from os import environ


# *************************************************************************************************

VOTRE_NUMERO_EQUIPE = 1
SIM = True
ADMIN = False
VOTRE_NUMERO_ROBOT = 7
    
if SIM:
    environ['ROS_MASTER_URI'] = f"http://localhost:113{70+VOTRE_NUMERO_EQUIPE if not ADMIN else 11}/"
else:
    environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(VOTRE_NUMERO_ROBOT)
    environ['ROS_IP'] = get_ip() # adresse IP de votre station de travail

rospy.init_node('dingo_controller', anonymous=True)
rate = rospy.Rate(50)

# *************************************************************************************************


# Fonction pour le calcul de l'orientation à partir d'un quaternion
def get_heading_from_quaternion(q):
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]
def wraptopi(angle):
    xwrap=np.remainder(angle, 2*np.pi)
    if np.abs(xwrap)>np.pi:
        xwrap -= 2*np.pi * np.sign(xwrap)
    return xwrap

# Laser scan subscriber callback
laser_msg = LaserScan()
def laser_scan_callback(msg):
    global laser_msg
    laser_msg = msg

# Realsense Pose subscriber callback
t265_msg = Pose()
def t265_callback(msg):
    global t265_msg
    t265_msg = msg.pose.pose
    pose = PoseStamped()

    pose.header.seq = 1
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"

    pose.pose.position.x = t265_msg.position.x
    pose.pose.position.y = t265_msg.position.y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = t265_msg.orientation.x
    pose.pose.orientation.y = t265_msg.orientation.y
    pose.pose.orientation.z = t265_msg.orientation.z
    pose.pose.orientation.w = t265_msg.orientation.w

# ROS subscribers et publishers
if SIM:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
else:
    cmd_drive_pub = rospy.Publisher('/mobile_manip/base/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
laser_sub = rospy.Subscriber('/scan', LaserScan, laser_scan_callback)
t265_sub = rospy.Subscriber('/mobile_manip/t265/odom/sample', Odometry, t265_callback)

## DÉPLACEMENT DU ROBOT
interwheel_distance = 0.434
left_wheel_radius = 0.049
right_wheel_radius = 0.049

def move_robot(linear, angular):
    vel_left  = (linear - angular * interwheel_distance / 2.0) / left_wheel_radius
    vel_right = (linear + angular * interwheel_distance / 2.0) / right_wheel_radius

    # Envoi des commandes au roues par topic ROS
    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    cmd_drive_pub.publish(cmd_drive_msg)

while(t265_msg.position.x == 0):
    print('no odom msg')
    time.sleep(0.1)

## INTERFACE
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

root = tk.Tk()

fig = Figure(figsize=(6,3))
ax = fig.add_subplot(111)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def update_plot():
    cap = wraptopi(get_heading_from_quaternion(t265_msg.orientation))
    x_pos = t265_msg.position.x
    y_pos = t265_msg.position.y

    obstacles_xy = []
    for i, r in enumerate(laser_msg.ranges):
        angle = cap - laser_msg.angle_min + i * laser_msg.angle_increment
        obstacles_xy.append([x_pos + r * math.cos(angle), y_pos + r * math.sin(angle)])
        # Arrête si un obstacle directement à l'avant à moins de 1m.
        #if i == 0 and r < 1.0 and r > 0.01:
        #    move_robot(0,0)
        
    ax.clear()
    ax.scatter(np.asarray(obstacles_xy)[:,0], np.asarray(obstacles_xy)[:,1], color='r')
    ax.scatter(x_pos, y_pos, color='g')
    ax.axis([-10, 10, -10, 10])
    ax.grid(True)
    canvas.draw()
    root.after(10, update_plot)

buttons_frame = tk.Frame(root)
buttons_frame.pack()

linear_speed = 0.25
angular_speed = 0.25

btn_forward = tk.Button(buttons_frame, text="Forward", command=lambda: move_robot(linear_speed, 0))
btn_left = tk.Button(buttons_frame, text="Left", command=lambda: move_robot(0, angular_speed))
btn_backward = tk.Button(buttons_frame, text="Backward", command=lambda: move_robot(-linear_speed, 0))
btn_right = tk.Button(buttons_frame, text="Right", command=lambda: move_robot(0, -angular_speed))
btn_stop = tk.Button(buttons_frame, text="Stop", command=lambda: move_robot(0,0))

btn_forward.grid(row=0, column=1)
btn_left.grid(row=1, column=0)
btn_stop.grid(row=1, column=1)
btn_right.grid(row=1, column=2)
btn_backward.grid(row=2, column=1)

update_plot()
root.mainloop()