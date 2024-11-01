#!/usr/bin/env python3

"""
The purpose of this file is for testing courses in the simulator without needing the autonomy GUI.
You can add waypoints with and without tags and these will get published to navigation.
"""

import sys

import numpy as np
import tkinter as tk
import time

from numpy._typing._array_like import NDArray

import rclpy
from navigation.context import Context, CostMap, Environment, ImageTargetsStore
from navigation.astar import AStar
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import tkinter as tk
import numpy as np
import time

import tkinter as tk
import numpy as np
import time

class SimWindow:
    def __init__(self, ctx: Context, rows=10, cols=10, cell_size=50, grid=None, start=None, end=None, speed=250):
        self.ctx = ctx

        self.root = tk.Tk()
        self.root.title("AStar Simulation Testing")
        self.number_padding = cell_size

        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.current_path_index = 0

        self.grid = np.zeros((self.rows, self.cols), dtype=int) if grid is None else grid
        self.grid_copy = np.copy(self.grid)
        self.start = start
        self.end = end
        self.speed = speed

        self.canvas = tk.Canvas(
            self.root, 
            width=cols*cell_size + self.number_padding,
            height=rows*cell_size + self.number_padding
        )

        self.canvas.pack()

        # Draw grid
        self.draw_grid()
        
        self.dragging = False
        self.start_x = None
        self.start_y = None
        self.curr_cell = (-1, -1)
        
        # Bind mouse events
        self.canvas.bind("<ButtonPress-1>", self.start_drag)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drag)
        self.canvas.bind_all("<q>", lambda event: self.root.destroy())

        # Add print button
        self.print_button = tk.Button(
            self.root,
            text="Run Simulation",
            command=self.run_astar
        )
        self.print_button.pack(pady=5)

    def run_astar(self):
        self.current_path_index = 0
        self.canvas.unbind("<ButtonPress-1>")
        self.canvas.unbind("<B1-Motion>")
        self.canvas.unbind("<ButtonRelease-1>")

        if not hasattr(self.ctx, 'env'):
            costmap = CostMap()
            costmap.origin = np.array([0,0])
            costmap.height = 15
            costmap.width = 10
            costmap.resolution = 1
            costmap.data = np.copy(self.grid)
            self.ctx.env = Environment(self.ctx, image_targets=ImageTargetsStore(self.ctx), cost_map=costmap)

        astar = AStar(np.array([0,0]), self.ctx)
        self.path_history = astar.a_star(self.start[::-1], self.end[::-1], debug=True)
        if self.path_history:
            self.pause = False
            if self.path_history:
                self.root.bind("<space>", self.toggle_animation)
            # Start the animation loop
            self.animate_path()

    def toggle_animation(self, event):
        self.pause = not self.pause
        if not self.pause:
            # Resume animation if we're unpausing
            self.animate_path()

    def animate_path(self):
        if self.pause: return
        
        if self.path_history and self.current_path_index < len(self.path_history):
            self.draw_grid()
            self.grid = np.copy(self.grid_copy)
            for i, coord in enumerate(self.path_history[self.current_path_index][1:-1]):
                self.grid[coord[0]][coord[1]] = 10
                
                x = coord[1] * self.cell_size + self.cell_size/2 + self.number_padding
                y = coord[0] * self.cell_size + self.cell_size/2 + self.number_padding
                
                self.fill_cell(coord[0], coord[1], 'blue')
                self.canvas.create_text(
                    x,
                    y,
                    text=str(i),
                    font=('Arial', self.cell_size // 2),
                    fill='white'
                )

            self.current_path_index += 1

            self.root.after(self.speed, self.animate_path)

    def draw_grid(self):
        self.canvas.delete("all")

        for row in range(self.rows):
            y = row * self.cell_size + self.cell_size/2 + self.number_padding
            self.canvas.create_text(
                self.number_padding/2, 
                y,
                text=str(row),
                font=('Arial', self.cell_size // 2)
            )

        # Draw column numbers
        for col in range(self.cols):
            x = col * self.cell_size + self.cell_size/2 + self.number_padding
            self.canvas.create_text(
                x,
                self.number_padding/2,
                text=str(col),
                font=('Arial', self.cell_size // 2)
            )
        
        # Fill cells based on array values
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid[row][col] == 1:  # Path cells
                    self.fill_cell(row, col, 'gray')
        
        # start cell
        if self.start is not None: self.fill_cell(self.start[1],self.start[0], 'green')

        # end cell
        if self.end is not None: self.fill_cell(self.end[1],self.end[0], 'red')

        # Draw vertical lines
        for i in range(self.cols + 1):
            x = i * self.cell_size + self.number_padding
            self.canvas.create_line(x, 0+self.number_padding, x, self.rows * self.cell_size + self.number_padding, width=2)
        
        # Draw horizontal lines
        for i in range(self.rows + 1):
            y = i * self.cell_size + self.number_padding
            self.canvas.create_line(0+self.number_padding, y, self.cols * self.cell_size + self.number_padding, y, width=2)
    
    def fill_cell(self, row, col, color='blue'):
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
        
        self.grid[row][col] = 1 - self.grid[row][col]
        self.curr_cell = (row, col)
        self.draw_grid()

    def on_drag(self, event):
        if self.dragging:
            col = min(max((event.x - self.number_padding) // self.cell_size, 0), self.cols - 1)
            row = min(max((event.y - self.number_padding) // self.cell_size, 0), self.rows - 1)
            
            if (row, col) != self.curr_cell:
                self.grid[row][col] = 1 - self.grid[row][col]
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
        self.declare_parameters("",[("search.traversable_cost", Parameter.Type.DOUBLE)])
        self.set_parameters([Parameter("search.traversable_cost", Parameter.Type.DOUBLE, 0.2)])

        start = np.array([3,3])
        end = np.array([10,15])

        sim = SimWindow(self.ctx, rows=20, cols=30, cell_size=30, start=start, end=end)
        path_history = sim.run()

        for path in path_history:
            temp = ""
            for coord in path:
                temp += str(coord) + " "
            self.get_logger().info(temp)

                



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
