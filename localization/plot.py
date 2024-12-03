import pandas as pd
import matplotlib.pyplot as plt

linearized_position = pd.read_csv("../long_linearized_position.csv")
heading = pd.read_csv

x = linearized_position["x"]
y = linearized_position["y"]

plt.figure(figsize=(8, 6))
plt.scatter(x,y,s=10)
plt.title("Tethered RTK test")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.savefig("11_18.png")