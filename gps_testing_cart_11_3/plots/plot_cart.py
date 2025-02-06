import pandas as pd
import matplotlib.pyplot as plt

linearized_position = pd.read_csv("linearized_position_cart.csv")

#print(linearized_position.columns)
x = linearized_position["x"]
y = linearized_position["y"]

plt.figure(figsize=(8, 6))
plt.scatter(x,y,s=10)
plt.title("Moving RTK test (tethered, none/float)")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.savefig("cart.png")