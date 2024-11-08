import pandas as pd
import matplotlib.pyplot as plt

heading = pd.read_csv("../heading.csv")
heading_status = pd.read_csv("../heading_fix_status.csv")

heading_deg = heading["heading"]
sec = heading["sec"]
status = heading_status["fix_status"]


plt.figure(figsize=(8, 6))
plt.scatter(sec,heading_deg,c=status,cmap=plt.cm.Accent,s=10)
plt.xlabel("time (s)")
plt.ylabel("heading (deg)")
plt.title("Heading test (fixed, No RTK, untethered)")

plt.savefig("heading.png")