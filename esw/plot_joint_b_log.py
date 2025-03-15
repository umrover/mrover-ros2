import csv
import matplotlib.pyplot as plt
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <csv_filename>")
        sys.exit(1)

    filename = sys.argv[1]

    throttle = []
    position = []
    velocity = []

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        # Skip the header
        next(reader, None)
        for row in reader:
            if len(row) < 3:
                continue
            thr = float(row[0])
            pos = float(row[1])
            vel = float(row[2])
            throttle.append(thr)
            position.append(pos)
            velocity.append(vel)

    x_axis = list(range(len(throttle)))

    # Create base axis for throttle
    fig, ax1 = plt.subplots(figsize=(10, 5))
    color1 = 'tab:blue'
    ax1.plot(x_axis, throttle, label="Throttle Cmd", color=color1)
    ax1.set_xlabel("Index")
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
    ax3.spines["right"].set_position(("outward", 60))  # Offset by 60 points
    ax3.plot(x_axis, velocity, label="Velocity", color=color3)
    ax3.set_ylabel("Velocity", color=color3)
    ax3.tick_params(axis='y', labelcolor=color3)

    # Combine legends from all axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    lines3, labels3 = ax3.get_legend_handles_labels()
    ax1.legend(lines1 + lines2 + lines3, labels1 + labels2 + labels3, loc='upper left')

    plt.title("Throttle Cmd, Position, and Velocity over Indices")
    plt.grid(True)
    plt.savefig("joint_b_log.png")
    plt.show()

if __name__ == "__main__":
    main()

