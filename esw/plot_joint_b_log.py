import csv
import matplotlib.pyplot as plt
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <csv_filename>")
        sys.exit(1)

    filename = sys.argv[1]

    epoch_times = []
    throttle = []
    position = []
    velocity = []

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 4:
                continue
            t_epoch = float(row[0])
            thr = float(row[1])
            pos = float(row[2])
            vel = float(row[3])
            epoch_times.append(t_epoch)
            throttle.append(thr)
            position.append(pos)
            velocity.append(vel)

    if not epoch_times:
        print("No valid data found.")
        sys.exit(1)

    # Convert epoch times to seconds relative to the start time
    start_time = epoch_times[0]
    x_axis = [t - start_time for t in epoch_times]

    # Create the first axis for throttle command
    fig, ax1 = plt.subplots(figsize=(10, 5))
    color1 = 'tab:blue'
    ax1.plot(x_axis, throttle, label="Throttle Cmd", color=color1)
    ax1.set_xlabel("Time (seconds)")
    ax1.set_ylabel("Throttle Cmd", color=color1)
    ax1.tick_params(axis='y', labelcolor=color1)

    # Create a twin axis for position
    ax2 = ax1.twinx()
    color2 = 'tab:green'
    ax2.plot(x_axis, position, label="Position", color=color2)
    ax2.set_ylabel("Position", color=color2)
    ax2.tick_params(axis='y', labelcolor=color2)

    # Create another twin axis for velocity and offset its spine
    ax3 = ax1.twinx()
    color3 = 'tab:red'
    ax3.spines["right"].set_position(("outward", 60))  # Offset axis by 60 points
    ax3.plot(x_axis, velocity, label="Velocity", color=color3)
    ax3.set_ylabel("Velocity", color=color3)
    ax3.tick_params(axis='y', labelcolor=color3)

    # Combine legends from all axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    lines3, labels3 = ax3.get_legend_handles_labels()
    ax1.legend(lines1 + lines2 + lines3, labels1 + labels2 + labels3, loc='upper left')

    plt.title("Throttle Cmd, Position, and Velocity over Time")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()

