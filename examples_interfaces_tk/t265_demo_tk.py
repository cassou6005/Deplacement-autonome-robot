import rospy
from jackal_msgs.msg import Drive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import numpy as np
import tkinter as tk
from tkinter import Button
import threading
import time
import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import pyplot as plt
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

pose_msg = Pose()
def pose_callback(msg):
    global pose_msg
    pose_msg = msg.pose.pose

def get_heading_from_quaternion(q):
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]

def wraptopi(angle):
    xwrap = np.remainder(angle, 2*np.pi)
    if np.abs(xwrap) > np.pi:
        xwrap -= 2*np.pi * np.sign(xwrap)
    return xwrap

cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
pose_sub = rospy.Subscriber('/mobile_manip/t265/odom/sample', Odometry, pose_callback)

interwheel_distance = 0.434
left_wheel_radius = 0.098/2
right_wheel_radius = 0.098/2

def move_robot(linear, angular):
    vel_left = (linear - angular * interwheel_distance / 2.0) / left_wheel_radius
    vel_right = (linear + angular * interwheel_distance / 2.0) / right_wheel_radius
    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    cmd_drive_pub.publish(cmd_drive_msg)


# Matplotlib Plotting Setup
fig = plt.Figure(figsize=(6, 3), tight_layout=True)
XY = []
rospy.init_node('dingo_controller', anonymous=True)
rate = rospy.Rate(50)
def update_plot():
    global fig, XY
    while True:
        cap = wraptopi(get_heading_from_quaternion(pose_msg.orientation))
        x_pos = pose_msg.position.x
        y_pos = pose_msg.position.y
        dx = math.cos(cap)
        dy = math.sin(cap)
        XY.append([x_pos, y_pos])
        
        # Plotting the position
        ax = fig.add_subplot(111)
        ax.cla()  # Clear previous plot
        ax.scatter(np.asarray(XY)[:, 0], np.asarray(XY)[:, 1], color='y')
        ax.arrow(x_pos, y_pos, dx, dy, color="#aa0088")
        ax.set_xlim([-10, 10])
        ax.set_ylim([-10, 10])
        ax.grid(True)
        canvas.draw_idle()
        plt.pause(0.1)

# Tkinter GUI Setup
root = tk.Tk()
root.title("Dingo Controller")

# Create Matplotlib Canvas for plotting
canvas_frame = tk.Frame(root)
canvas_frame.grid(row=0, column=0, columnspan=4)
canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
canvas.get_tk_widget().pack()

# Create Buttons for movement controls
btn_up = Button(root, text="Up", command=lambda: move_robot(0.25, 0))
btn_up.grid(row=1, column=1)

btn_left = Button(root, text="Left", command=lambda: move_robot(0, 0.25))
btn_left.grid(row=2, column=0)

btn_down = Button(root, text="Down", command=lambda: move_robot(-0.25, 0))
btn_down.grid(row=3, column=1)

btn_right = Button(root, text="Right", command=lambda: move_robot(0, -0.25))
btn_right.grid(row=2, column=2)

btn_stop = Button(root, text="Stop", command=lambda: move_robot(0, 0))
btn_stop.grid(row=2, column=1)

btn_eff = Button(root, text="Clear", command=lambda: XY.clear())
btn_eff.grid(row=0, column=1)

# Thread for updating the plot
def plot_thread():
    update_plot()

# Start the plot update thread
thread = threading.Thread(target=plot_thread)
thread.daemon = True
thread.start()

# Start Tkinter GUI main loop
root.mainloop()
