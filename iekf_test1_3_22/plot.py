import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

pose_db = pd.read_csv("pose.csv")
linearized_db = pd.read_csv("linearized_position.csv")
mag_heading_db = pd.read_csv("mag_heading.csv")

pose_x = pose_db["x"]
pose_y = pose_db["y"]
time_sec = pose_db["sec"]
time_nanosec = pose_db["nanosec"]

time = time_sec + 1e-09 * time_nanosec

linearized_x = linearized_db["x"]
linearized_y = linearized_db["y"]

plt.scatter(pose_x,pose_y, 1)
plt.title("iekf filtered position")
plt.savefig("pose_pos.png")

plt.clf()
plt.scatter(linearized_x, linearized_y, 1)
plt.title("position raw")
plt.savefig("linearized_pos.png")

quaternions = np.column_stack([pose_db["orientation_x"], pose_db["orientation_y"], pose_db["orientation_z"], pose_db["orientation_w"]])
euler = np.zeros((quaternions.shape[0], 3))

for i in range(quaternions.shape[0]):
    r = R.from_quat(quaternions[i,:])
    euler_angles = r.as_euler('xyz', degrees=True)
    euler[i] = euler_angles

plt.clf()

plt.figure(figsize=(60,60))
fig, ax = plt.subplots(3,ncols=1,sharey=True)
ax[0].scatter(time, euler[:,0], color="red", s=1)
ax[0].set_ylabel("roll")
ax[1].scatter(time, euler[:,1], color="blue", s=1)
ax[1].set_ylabel("pitch")
ax[2].scatter(time, euler[:,2], color="green", s=1)
ax[2].set_xlabel("time")
ax[2].set_ylabel("yaw")

plt.savefig("euler.png")

plt.clf()
plt.scatter(time, euler[:,2], s=1)
plt.title("iekf filtered heading")
plt.savefig("filtered_heading.png")

heading = mag_heading_db["heading"]
time_sec = mag_heading_db["sec"]
time_nanosec = mag_heading_db["nanosec"]
time = time_sec + 1e-09 * time_nanosec

heading = 90 - heading
heading[heading < -180] = 360 + heading

plt.clf()
plt.scatter(time, heading, s=1)
plt.title("mag heading raw")
plt.savefig("raw_heading.png")

velocity_db = pd.read_csv("velocity.csv")
time_sec = velocity_db["sec"]
time_nanosec = velocity_db["nanosec"]
time = time_sec + 1e-09 * time_nanosec
velocity_x = velocity_db["x"]
velocity_y = velocity_db["y"]

integrated_pos = np.zeros((len(time), 3))
integrated_pos[0,:] = [linearized_x[0], linearized_y[0], 0]
for i in range(1, len(time)):
    dt = time[i] - time[i - 1]
    integrated_pos[i,0] = integrated_pos[i - 1, 0] + velocity_x[i - 1] * dt
    integrated_pos[i,1] = integrated_pos[i - 1, 1] + velocity_y[i - 1] * dt
    integrated_pos[i,2] = integrated_pos[i - 1, 2]
    

plt.clf()
plt.scatter(linearized_x, linearized_y, s=1, label="raw position")
plt.scatter(integrated_pos[:,0], integrated_pos[:,1], s=1, label="integrated position")
plt.title("integrated vs raw position")
plt.legend()
plt.savefig("integrated_pos.png")
    



