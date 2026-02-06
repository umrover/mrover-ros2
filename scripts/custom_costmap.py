#!/usr/bin/env python3
import sys
import tkinter as tk
import rclpy
from rclpy.node import Node
from rclpy import Parameter
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import numpy as np


class GridUI:
    """
    Creates a Tkinter UI with an n×n grid.
    Clicking and dragging paints cells black.
    The middle cell (origin) is marked blue.
    Press the spacebar to cycle brush size (1x1, 2x2, 3x3).
    """

    def __init__(self, master, n=32, cell_size=20):
        self.master = master
        self.n = n
        self.cell_size = cell_size
        self.canvas_size = n * cell_size
        self.brush_size = 1  # Default brush is 1x1
        self.origin = (n // 2, n // 2)  # Mark the center cell as the origin

        # Create canvas and clear button
        self.canvas = tk.Canvas(master, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.pack(pady=10)
        self.clear_button = tk.Button(master, text="Clear", command=self.clear_grid)
        self.clear_button.pack(pady=5)

        # Initialize grid state (0: free, 1: painted)
        self.grid = [[0 for _ in range(n)] for _ in range(n)]
        self.rectangles = np.array([[None for _ in range(n)] for _ in range(n)])
        for i in range(n):
            for j in range(n):
                x1 = j * cell_size
                y1 = i * cell_size
                x2 = x1 + cell_size
                y2 = y1 + cell_size
                rect = self.canvas.create_rectangle(x1, y1, x2, y2, outline="gray", fill="white")
                self.rectangles[i][j] = rect

        # Mark the origin cell blue
        oi, oj = self.origin
        self.canvas.itemconfig(self.rectangles[oi][oj], fill="blue")

        # Bind mouse events for painting and spacebar for brush toggle
        self.canvas.bind("<Button-1>", self.on_mouse_drag)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        master.bind("<space>", self.toggle_brush)

    def toggle_brush(self, event):
        """Cycle the brush size between 1x1, 2x2, and 3x3."""
        if self.brush_size == 1:
            self.brush_size = 2
        elif self.brush_size == 2:
            self.brush_size = 3
        else:
            self.brush_size = 1
        print(f"Brush size set to {self.brush_size}x{self.brush_size}")

    def on_mouse_drag(self, event):
        """Paint cells black using a centered brush of current size."""
        # Determine the central cell from the mouse coordinates
        i = event.y // self.cell_size
        j = event.x // self.cell_size

        # Calculate offset so that the brush is centered
        offset = self.brush_size // 2

        # Paint a brush_size x brush_size block centered at (i, j)
        for di in range(-offset, -offset + self.brush_size):
            for dj in range(-offset, -offset + self.brush_size):
                ii = i + di
                jj = j + dj
                if 0 <= ii < self.n and 0 <= jj < self.n:
                    # Skip the origin cell so it remains blue
                    if (ii, jj) == self.origin:
                        continue
                    if self.grid[ii][jj] == 0:
                        self.grid[ii][jj] = 1
                        self.canvas.itemconfig(self.rectangles[ii][jj], fill="black")

    def clear_grid(self):
        """Reset grid state and update cell colors. Origin remains blue."""
        for i in range(self.n):
            for j in range(self.n):
                self.grid[i][j] = 0
                self.canvas.itemconfig(self.rectangles[i][j], fill="white")
        # Re-mark the origin cell as blue
        oi, oj = self.origin
        self.canvas.itemconfig(self.rectangles[oi][oj], fill="blue")

    def get_flat_grid(self):
        """
        Returns the grid as a flat list.
        Free cells are 0 and painted cells are 100.
        """
        flat = []
        transposed = [[self.grid[j][i] for j in range(len(self.grid))] for i in range(len(self.grid[0]))]
        for row in transposed:
            for cell in row:
                flat.append(100 if cell == 1 else 0)
        return flat


class CustomCostmapNode(Node):
    """
    ROS 2 node that publishes a costmap as an OccupancyGrid message.
    """

    def __init__(self, ui, n):
        super().__init__("custom_costmap")
        # Declare parameters with provided default values
        self.declare_parameter("size", n)
        self.declare_parameter("resolution", 0.5)
        self.publisher_ = self.create_publisher(OccupancyGrid, "custom_costmap", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.ui = ui

        # Read parameters
        self.n = self.get_parameter("size").value
        self.resolution = self.get_parameter("resolution").value

        # Set up the OccupancyGrid info fields
        self.occupancy_info = MapMetaData()
        self.occupancy_info.resolution = self.resolution  # meters per cell
        self.occupancy_info.width = self.n
        self.occupancy_info.height = self.n

        # Set the origin of the map to be centered (in meters)
        origin = Pose()
        origin.position.x = -self.n * self.resolution / 2
        origin.position.y = -self.n * self.resolution / 2
        origin.position.z = 0.0
        origin.orientation.x = 0.0
        origin.orientation.y = 0.0
        origin.orientation.z = 0.0
        origin.orientation.w = 1.0
        self.occupancy_info.origin = origin

    def timer_callback(self):
        """Publish the OccupancyGrid message built from the current UI grid."""
        msg = OccupancyGrid()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        msg.header = header

        msg.info = self.occupancy_info
        msg.data = self.ui.get_flat_grid()

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=sys.argv)

    # Create a temporary node to get the parameter values before creating the UI.
    temp_node = Node("temp_node")
    temp_node.declare_parameter("size", 32)
    temp_node.declare_parameter("resolution", 0.5)
    grid_size = temp_node.get_parameter("size").value
    temp_node.destroy_node()

    # Create the Tkinter root and UI using the parameter value.
    root = tk.Tk()
    root.title("Customizable Costmap")
    ui = GridUI(root, n=grid_size, cell_size=20)

    # Create the ROS 2 node with the UI.
    node = CustomCostmapNode(ui, n=grid_size)

    try:
        while rclpy.ok():
            root.update()
            rclpy.spin_once(node, timeout_sec=0.1)
    except tk.TclError:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
