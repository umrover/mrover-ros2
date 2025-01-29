import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data_raw_df = pd.read_csv("zed_imu_data_raw.csv")

time_sec = data_raw_df["sec"]
time_nanosec = data_raw_df["nanosec"]

time = time_sec + 1e-09 * time_nanosec

angular_vel_x = data_raw_df["angular_vel_x"]
angular_vel_y = data_raw_df["angular_vel_y"]
angular_vel_z = data_raw_df["angular_vel_z"]

plt.figure(figsize=(30,60))
fig, ax = plt.subplots(3,ncols=1,sharey=True)
ax[0].plot(time, angular_vel_x, color="red")
ax[0].set_ylabel("x")
ax[1].plot(time, angular_vel_y, color="blue")
ax[1].set_ylabel("y")
ax[2].plot(time, angular_vel_z, color="green")
ax[2].set_xlabel("time")
ax[2].set_ylabel("z")
plt.suptitle("Angular velocity")
plt.savefig("angular_vel.png")

# mag_df = pd.read_csv("zed_imu_mag.csv")

# time_mag_sec = mag_df["sec"]
# time_mag_nanosec = mag_df["nanosec"]
# time_mag = time_mag_sec + 1e-09 * time_mag_nanosec

# mag_field_x = mag_df["mag_field_x"]
# mag_field_y = mag_df["mag_field_y"]
# mag_field_z = mag_df["mag_field_z"]

# heading = np.arctan2(mag_field_y, mag_field_x) * 180 / np.pi

# plt.clf()
# plt.figure(figsize=(20,15))
# plt.scatter(time_mag, heading,s=1)
# plt.xlabel("time")
# plt.ylabel("heading (deg)")
# plt.title("Heading calculated from magnetic field")
# plt.savefig("heading_from_mag.png")

# should only be valid if size of the vector is close to g

accel_x = data_raw_df["accel_x"]
accel_y = data_raw_df["accel_y"]
accel_z = data_raw_df["accel_z"]

plt.clf()
plt.figure(figsize=(20,15))
plt.plot(time, accel_x, label="x accel")
plt.plot(time, accel_y, label="y accel")
plt.plot(time, accel_z, label="z accel")
plt.legend()
plt.savefig("accel.png")

# pitch from accelerometer

pitch = np.arcsin(accel_x / np.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)) * 180 / np.pi

plt.clf()
plt.figure(figsize=(20, 15))
plt.scatter(time, pitch, s=1)
plt.xlabel("time")
plt.ylabel("pitch (deg)")
plt.title("pitch calculated from accelerometer")
plt.savefig("pitch_from_accel.png")

# roll from accelerometer (add a small percentage of accelerometer x axis to resolve 0/0 issue)
roll = np.zeros(accel_y.shape[0])
for i in range(accel_y.shape[0]):

    miu = 0.05
    roll[i] = np.arctan2(accel_y[i], np.sqrt(accel_z[i] * accel_z[i] + miu * accel_x[i] * accel_x[i])) * 180 / np.pi

# roll = np.arcsin2(accel_y, accel_z) * 180 / np.pi
plt.clf()
plt.figure(figsize=(20, 15))
plt.scatter(time, roll, s=1)
plt.xlabel("time")
plt.ylabel("roll (deg)")
plt.title("roll calculated from accelerometer")
plt.savefig("roll_from_accel.png")

# orientation_x = data_raw_df["orientation_x"]
# orientation_y = data_raw_df["orientation_y"]
# orientation_z = data_raw_df["orientation_z"]
# orientation_w = data_raw_df["orientation_w"]

# heading_from_orientation = np.arccos(orientation_w)

# plt.clf()
# plt.figure(figsize=(20, 15))
# plt.scatter(time, heading_from_orientation, s=1)
# plt.xlabel("time")
# plt.ylabel("heading (deg)")
# plt.title("heading calculated from quaternion")
# plt.savefig("yaw_from_quat.png")

# pitch = np.arctan2(-accel_x, np.sqrt(accel_y * accel_y * accel_z * accel_z)) * 57.3

# plt.clf()
# plt.figure(figsize=(20, 15))
# plt.scatter(time, pitch, s=1)
# plt.xlabel("time")
# plt.ylabel("pitch (deg)")
# plt.title("pitch calculated from accelerometer")
# plt.savefig("pitch_from_accel.png")


