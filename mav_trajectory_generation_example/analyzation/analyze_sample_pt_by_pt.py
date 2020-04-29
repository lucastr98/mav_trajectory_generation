import rosbag
import numpy as np
import tf
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig, axes = plt.subplots(3, 3)

bag = rosbag.Bag('bagname.bag')

# odometry of the drone
odometry_roll = []
odometry_pitch = []
odometry_pos_x = []
odometry_pos_y = []
odometry_pos_z = []
odometry_vel_x = []
odometry_vel_y = []
odometry_vel_z = []
odometry_time = []
for topic, msg, t in bag.read_messages(topics='/peregrine/odometry_sensor1/odometry'):
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    odometry_roll.append(roll)
    odometry_pitch.append(pitch)
    odometry_pos_x.append(msg.pose.pose.position.x)
    odometry_pos_y.append(msg.pose.pose.position.y)
    odometry_pos_z.append(msg.pose.pose.position.z)
    mat11 = 1 - 2*y*y - 2*z*z
    mat12 = 2*x*y - 2*z*w
    mat13 = 2*x*z + 2*y*w
    mat21 = 2*x*y + 2*z*w
    mat22 = 1 - 2*x*x - 2*z*z
    mat23 = 2*y*z - 2*x*w
    mat31 = 2*x*z - 2*y*w
    mat32 = 2*y*z + 2*x*w
    mat33 = 1 - 2*x*x - 2*y*y
    v_x_body = msg.twist.twist.linear.x
    v_y_body = msg.twist.twist.linear.y
    v_z_body = msg.twist.twist.linear.z
    odometry_vel_x.append(mat11*v_x_body + mat12*v_y_body + mat13*v_z_body)
    odometry_vel_y.append(mat21*v_x_body + mat22*v_y_body + mat23*v_z_body)
    odometry_vel_z.append(mat31*v_x_body + mat32*v_y_body + mat33*v_z_body)
    odometry_time.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
axes[0][0].plot(odometry_time, odometry_pos_x, '.', label='odometry', c='b')
axes[1][0].plot(odometry_time, odometry_pos_y, '.', label='odometry', c='b')
axes[2][0].plot(odometry_time, odometry_pos_z, '.', label='odometry', c='b')
axes[0][1].plot(odometry_time, odometry_vel_x, '.', label='odometry', c='b')
axes[1][1].plot(odometry_time, odometry_vel_y, '.', label='odometry', c='b')
axes[2][1].plot(odometry_time, odometry_vel_z, '.', label='odometry', c='b')
axes[0][2].plot(odometry_time, odometry_roll, '.', label='odometry', c='b')
axes[1][2].plot(odometry_time, odometry_pitch, '.', label='odometry', c='b')

# command sent to drone
command_roll = []
command_pitch = []
command_time = []
for topic, msg, t in bag.read_messages(topics='/peregrine/command/roll_pitch_yawrate_thrust'):
    command_roll.append(msg.roll)
    command_pitch.append(msg.pitch)
    command_time.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
axes[0][2].plot(command_time, command_roll, '.', label='command', c='r')
axes[1][2].plot(command_time, command_pitch, '.', label='command', c='r')

# trajectory sent to drone
trajectory_pos_x = []
trajectory_pos_y = []
trajectory_pos_z = []
trajectory_vel_x = []
trajectory_vel_y = []
trajectory_vel_z = []
trajectory_time = []
for topic, msg, t in bag.read_messages(topics='/peregrine/command/trajectory'):
    trajectory_pos_x.append(msg.points[0].transforms[0].translation.x)
    trajectory_pos_y.append(msg.points[0].transforms[0].translation.y)
    trajectory_pos_z.append(msg.points[0].transforms[0].translation.z)
    trajectory_vel_x.append(msg.points[0].velocities[0].linear.x)
    trajectory_vel_y.append(msg.points[0].velocities[0].linear.y)
    trajectory_vel_z.append(msg.points[0].velocities[0].linear.z)
    trajectory_time.append(msg.points[0].time_from_start.secs +
                           msg.points[0].time_from_start.nsecs/1e9 +
                           msg.header.stamp.secs +
                           msg.header.stamp.nsecs/1e9)
axes[0][0].plot(trajectory_time, trajectory_pos_x, '.', label='command', c='r')
axes[1][0].plot(trajectory_time, trajectory_pos_y, '.', label='command', c='r')
axes[2][0].plot(trajectory_time, trajectory_pos_z, '.', label='command', c='r')
axes[0][1].plot(trajectory_time, trajectory_vel_x, '.', label='command', c='r')
axes[1][1].plot(trajectory_time, trajectory_vel_y, '.', label='command', c='r')
axes[2][1].plot(trajectory_time, trajectory_vel_z, '.', label='command', c='r')

axes[0][0].set(title='position x', xlabel='time [s]', ylabel='position [m]')
axes[1][0].set(title='position y', xlabel='time [s]', ylabel='position [m]')
axes[2][0].set(title='position z', xlabel='time [s]', ylabel='position [m]')
axes[0][1].set(title='linear velocity x', xlabel='time [s]', ylabel='velocity [m/s]')
axes[1][1].set(title='linear velocity y', xlabel='time [s]', ylabel='velocity [m/s]')
axes[2][1].set(title='linear velocity z', xlabel='time [s]', ylabel='velocity [m/s]')
axes[0][2].set(title='roll', xlabel='time [s]', ylabel='angle [rad]')
axes[1][2].set(title='pitch', xlabel='time [s]', ylabel='angle [rad]')
axes[0][0].grid()
axes[1][0].grid()
axes[2][0].grid()
axes[0][1].grid()
axes[1][1].grid()
axes[2][1].grid()
axes[0][2].grid()
axes[1][2].grid()
axes[0][0].legend(loc=0)
axes[1][0].legend(loc=0)
axes[2][0].legend(loc=0)
axes[0][1].legend(loc=0)
axes[1][1].legend(loc=0)
axes[2][1].legend(loc=0)
axes[0][2].legend(loc=0)
axes[1][2].legend(loc=0)

fig.delaxes(axes[2,2])

plt.show()
