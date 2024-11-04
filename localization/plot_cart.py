import pandas as pd
import matplotlib.pyplot as plt

linearized_position = pd.read_csv("../linearized_position_cart.csv")

#print(linearized_position.columns)
x = linearized_position["x"]
y = linearized_position["y"]

plt.scatter(x,y)
plt.savefig("cart.png")