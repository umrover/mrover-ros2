import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

arrow = u'$\u27A4$'

linearized_position = pd.read_csv("long_linearized_position.csv")
heading_df = pd.read_csv("long_heading.csv")
heading_fix_status_df = pd.read_csv("long_heading_fix_status.csv")
heading_df.reset_index()
heading_fix_status_df.reset_index()

x = linearized_position["x"]
y = linearized_position["y"]

plt.figure(figsize=(30, 20))
plt.plot(x,y,color='black')

for index, row in heading_df.iterrows():
    corresponding_row_pos = linearized_position.loc[linearized_position['sec'] == row['sec']]
    corresponding_row_heading_fix = heading_fix_status_df.loc[heading_fix_status_df['sec'] == row['sec']]

    heading_x = corresponding_row_pos['x']
    heading_y = corresponding_row_pos['y']
    heading_fix = corresponding_row_heading_fix['fix']

    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(-1 * (row['heading'] - 90))

    if (heading_fix.item() == 2):
        plt.plot(heading_x, heading_y, marker=rotated_marker, markersize=10, color='blue')
    else:
        plt.plot(heading_x, heading_y, marker=rotated_marker, markersize=10, color='orange')
    

    
custom_legend = [Line2D([0], [0], color='black', label='path'),
                               Line2D([0], [0], marker=mpl.markers.MarkerStyle(marker=arrow), markersize=10, color='blue', label='heading fixed'),
                               Line2D([0], [0], marker=mpl.markers.MarkerStyle(marker=arrow), markersize=10, color='orange', label='heading float')]





plt.title("Tethered RTK test")
plt.legend(handles=custom_legend)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.savefig("11_18.png")