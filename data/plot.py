import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV file
data = pd.read_csv("motor_encoder_data.csv")
data = data.dropna()

# Extract columns
time = np.array(data["time"].values)
throttle = np.array(data["throttle"].values)
position = np.array(data["pos"].values)
velocity = np.array(data["vel"].values)

t_min, t_max = 45.5, 46  # Adjust these values as needed
mask = (time >= t_min) & (time <= t_max)

# Apply mask
time_filtered = time[mask]
throttle_filtered = throttle[mask]
position_filtered = position[mask]
velocity_filtered = velocity[mask]

# Plot
plt.figure(figsize=(10, 5))
plt.plot(time_filtered, throttle_filtered, label="Throttle", color="b", linestyle="-")
plt.plot(time_filtered, position_filtered, label="Position", color="r", linestyle="-")
# plt.plot(time_filtered, velocity_filtered, label="Velocity", color="g", linestyle="-")

plt.xlabel("Time")
plt.ylabel("Value")
plt.title("Stuff")
plt.legend()
plt.grid(True)
plt.show()

# Scale throttle to max velocity
# max_velocity = 100
# print(max_velocity)
# scaled_throttle = throttle * max_velocity
#
# # Compute difference
# diff = scaled_throttle - velocity
#
# # Plot Throttle vs Velocity
# plt.figure(figsize=(10, 5))
# plt.subplot(2, 1, 1)
# plt.plot(time, scaled_throttle, label="Scaled Throttle")
# plt.plot(time, velocity, label="Velocity", linestyle='dashed')
# plt.xlabel("Time")
# plt.ylabel("Value")
# plt.legend()
# plt.title("Throttle vs Velocity")
#
# # Plot Difference
# plt.subplot(2, 1, 2)
# plt.plot(time, diff, label="Difference", color='red')
# plt.xlabel("Time")
# plt.ylabel("Difference")
# plt.legend()
# plt.title("Difference between Scaled Throttle and Velocity")
#
# plt.tight_layout()
# plt.show()
