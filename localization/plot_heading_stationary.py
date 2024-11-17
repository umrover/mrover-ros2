import pandas as pd
import matplotlib.pyplot as plt

heading = pd.read_csv("../heading_stationary.csv")
heading_status = pd.read_csv("../heading_stationary_fix_status.csv")

heading_deg = heading["heading"]
sec = heading["sec"]
status = heading_status["fix_status"]


plt.figure(figsize=(8, 6))
plt.scatter(sec,heading_deg,s=10)
plt.xlabel("time (s)")
plt.ylabel("heading (deg)")
plt.title("Stationary heading test (fixed, No RTK, untethered)")

plt.savefig("stationary_heading.png")