#!/usr/bin/env python3

"""
The purpose of this file is for testing courses in the simulator without needing the autonomy GUI.
You can add waypoints with and without tags and these will get published to navigation.
"""

import sys
import rclpy
from navigation.context import Context, CostMap, Environment, ImageTargetsStore
from navigation.astar import AStar
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tkinter as tk
import numpy as np


class SimWindow:
    def __init__(
        self,
        ctx: Context,
        rows=10,
        cols=10,
        cell_size=50,
        grid=None,
        start=None,
        end=None,
        speed=250,
    ):
        self.ctx = ctx
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.number_padding = cell_size

        # convert to (row, col) format
        self.start = start[::-1]
        self.end = end[::-1]

        self.speed = speed
        self.root = tk.Tk()
        self.root.title("AStar Simulation Testing")
        self.grid = np.zeros((self.rows, self.cols), dtype=int) if grid is None else grid
        self.grid_copy = np.copy(self.grid)

        self.canvas = tk.Canvas(
            self.root,
            width=cols * cell_size + self.number_padding,
            height=rows * cell_size + self.number_padding,
        )

        self.canvas.pack()
        self.draw_grid()

        self.dragging = False
        self.start_x = None
        self.start_y = None
        self.curr_cell = (-1, -1)

        self.canvas.bind("<ButtonPress-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drag)
        self.canvas.bind_all("<q>", lambda event: self.root.destroy())

        self.current_path_index = 0
        self.print_button = tk.Button(self.root, text="Run Simulation", command=self.run_astar)
        self.print_button.pack(pady=5)

    def run_astar(self):
        self.current_path_index = 0
        self.canvas.unbind("<ButtonPress-1>")
        self.canvas.unbind("<B1-Motion>")
        self.canvas.unbind("<ButtonRelease-1>")

        if not hasattr(self.ctx, "env"):
            costmap = CostMap()
            costmap.origin = np.array([0, 0])
            costmap.height = self.rows
            costmap.width = self.cols
            costmap.resolution = 1
            costmap.data = np.copy(self.grid).T
            self.ctx.env = Environment(self.ctx, image_targets=ImageTargetsStore(self.ctx), cost_map=costmap)
            # self.ctx.node.get_logger().info(costmap.data)
        astar = AStar(self.ctx)
        self.path_history = astar.a_star(self.start[::-1], self.end[::-1], context=self.ctx, debug=True)
        if self.path_history:
            self.pause = False
            if self.path_history:
                self.root.bind("<space>", self.toggle_animation)
            self.animate_path()

    def toggle_animation(self, event):
        self.pause = not self.pause
        if not self.pause:
            self.animate_path()

    def animate_path(self):
        if self.pause:
            return

        if self.path_history and self.current_path_index < len(self.path_history):
            self.draw_grid()
            self.grid = np.copy(self.grid_copy)

            for i, coord in enumerate(self.path_history[self.current_path_index][1:-1]):
                row, col = coord[1], coord[0]
                self.grid[row][col] = 10

                self.fill_cell(row, col, "blue")

                # converts the coordinates along the path to x, y in the canvas
                x = col * self.cell_size + self.cell_size / 2 + self.number_padding
                y = row * self.cell_size + self.cell_size / 2 + self.number_padding
                self.canvas.create_text(x, y, text=str(i), font=("Arial", self.cell_size // 2), fill="white")

            self.current_path_index += 1

            self.root.after(self.speed, self.animate_path)

    def draw_grid(self):
        self.canvas.delete("all")

        for row in range(self.rows):
            y = row * self.cell_size + self.cell_size / 2 + self.number_padding
            self.canvas.create_text(
                self.number_padding / 2,
                y,
                text=str(row),
                font=("Arial", self.cell_size // 2),
            )

        for col in range(self.cols):
            x = col * self.cell_size + self.cell_size / 2 + self.number_padding
            self.canvas.create_text(
                x,
                self.number_padding / 2,
                text=str(col),
                font=("Arial", self.cell_size // 2),
            )

        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid[row][col] == 10000:  # Path cells
                    self.fill_cell(row, col, "gray")

        if self.start is not None:
            self.fill_cell(self.start[0], self.start[1], "green")

        if self.end is not None:
            self.fill_cell(self.end[0], self.end[1], "red")

        for i in range(self.cols + 1):
            x = i * self.cell_size + self.number_padding
            self.canvas.create_line(
                x,
                0 + self.number_padding,
                x,
                self.rows * self.cell_size + self.number_padding,
                width=2,
            )

        for i in range(self.rows + 1):
            y = i * self.cell_size + self.number_padding
            self.canvas.create_line(
                0 + self.number_padding,
                y,
                self.cols * self.cell_size + self.number_padding,
                y,
                width=2,
            )

    # fills in a cell given a row and col
    def fill_cell(self, row, col, color="blue"):
        x1 = col * self.cell_size + self.number_padding
        y1 = row * self.cell_size + self.number_padding
        x2 = x1 + self.cell_size
        y2 = y1 + self.cell_size
        self.canvas.create_rectangle(x1, y1, x2, y2, fill=color)

    def start_drag(self, event):
        self.dragging = True
        self.start_x = event.x
        self.start_y = event.y

        col = min(max((event.x - self.number_padding) // self.cell_size, 0), self.cols - 1)
        row = min(max((event.y - self.number_padding) // self.cell_size, 0), self.rows - 1)

        self.grid[row][col] = 10000 - self.grid[row][col]
        self.curr_cell = (row, col)
        self.draw_grid()

    def on_drag(self, event):
        if self.dragging:
            col = min(max((event.x - self.number_padding) // self.cell_size, 0), self.cols - 1)
            row = min(max((event.y - self.number_padding) // self.cell_size, 0), self.rows - 1)

            if (row, col) != self.curr_cell:
                self.grid[row][col] = 10000 - self.grid[row][col]
                self.curr_cell = (row, col)
                self.draw_grid()

    def stop_drag(self, event):
        self.dragging = False
        self.grid_copy = np.copy(self.grid)

    def run(self):
        self.root.mainloop()
        return self.path_history


class AStarDebug(Node):
    ctx: Context

    def __init__(self, ctx: Context) -> None:
        super().__init__("astar_debug")

        self.ctx = ctx
        self.ctx.node = self
        self.declare_parameters("", [("search.traversable_cost", Parameter.Type.DOUBLE)])
        self.set_parameters([Parameter("search.traversable_cost", Parameter.Type.DOUBLE, 0.2)])

        self.declare_parameters("", [("search.angle_thresh", Parameter.Type.DOUBLE)])
        self.set_parameters([Parameter("search.angle_thresh", Parameter.Type.DOUBLE, 0.0872665)])

        self.declare_parameters("", [("costmap.costmap_thresh", Parameter.Type.DOUBLE)])
        self.set_parameters([Parameter("costmap.costmap_thresh", Parameter.Type.DOUBLE, 0.4)])

        start = np.array([0, 1])
        end = np.array([25, 18])

        sim = SimWindow(self.ctx, rows=30, cols=30, cell_size=30, start=start, end=end, speed=10)
        path_history = sim.run()

        self.get_logger().info(f"{sim.grid}")
        self.get_logger().info(f"Total iterations: {len(path_history)}")


if __name__ == "__main__":
    try:
        rclpy.init(args=sys.argv)
        context = Context()
        node = AStarDebug(context)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
