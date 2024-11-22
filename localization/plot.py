import pandas as pd
import matplotlib.pyplot as plt

linearized_position = pd.read_csv("../linearized_position_11_18.csv")

x = linearized_position["x"]
y = linearized_position["y"]

plt.figure(figsize=(8, 6))
plt.scatter(x,y,s=10)
plt.title("Tethered RTK test")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.savefig("11_18.png")