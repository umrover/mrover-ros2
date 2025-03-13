import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

pose_db = pd.read_csv("pose.csv")
linearized_db = pd.read_csv("linearized_position.csv")

pose_x = pose_db["x"]
pose_y = pose_db["y"]
time_sec = pose_db["sec"]
time_nanosec = pose_db["nanosec"]

time = time_sec + 1e-09 * time_nanosec

linearized_x = linearized_db["x"]
linearized_y = linearized_db["y"]

plt.scatter(pose_x,pose_y, 1)
plt.savefig("pose_pos.png")

plt.clf()
plt.scatter(linearized_x, linearized_y, 1)
plt.savefig("linearized_pos.png")



quaternions = np.column_stack([pose_db["orientation_x"], pose_db["orientation_y"], pose_db["orientation_z"], pose_db["orientation_w"]])
euler = np.zeros((quaternions.shape[0], 3))

for i in range(quaternions.shape[0]):
    r = R.from_quat(quaternions[i,:])
    euler_angles = r.as_euler('xyz', degrees=True)
    euler[i] = euler_angles

plt.clf()

plt.figure(figsize=(30,60))
fig, ax = plt.subplots(3,ncols=1,sharey=True)
ax[0].scatter(time, euler[:,0], color="red", s=1)
ax[0].set_ylabel("roll")
ax[1].scatter(time, euler[:,1], color="blue", s=1)
ax[1].set_ylabel("pitch")
ax[2].scatter(time, euler[:,2], color="green", s=1)
ax[2].set_xlabel("time")
ax[2].set_ylabel("yaw")

plt.savefig("euler.png")


