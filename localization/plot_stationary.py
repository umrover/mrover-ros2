import pandas as pd
import matplotlib.pyplot as plt

linearized_position = pd.read_csv("../linearized_position_stationary.csv")

x = linearized_position["x"]
y = linearized_position["y"]

plt.figure(figsize=(8, 6))
plt.scatter(x,y,s=10)
plt.title("Stationary RTK test (fixed)")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.savefig("stationary.png")