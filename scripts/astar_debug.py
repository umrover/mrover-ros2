#!/usr/bin/env python3

"""
The purpose of this file is for testing courses in the simulator without needing the autonomy GUI.
You can add waypoints with and without tags and these will get published to navigation.
"""

import sys

import numpy as np
import tkinter as tk
import time

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
    def __init__(self, rows=10, cols=10, cell_size=50, grid=None, path_history=None, start=None, end=None):
        self.root = tk.Tk()
        self.root.title("AStar Simulation Testing")

        self.number_padding = 20

        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.path_history = path_history
        self.current_path_index = 0
        self.current_coord_index = 0

        self.grid = np.zeros((self.rows, self.cols), dtype=int) if grid is None else grid
        self.grid_copy = np.copy(self.grid)
        self.start = start
        self.end = end

        self.canvas = tk.Canvas(
            self.root, 
            width=cols*cell_size + self.number_padding,
            height=rows*cell_size + self.number_padding
        )

        self.canvas.pack()

        # Draw grid
        self.draw_grid()
        
        if self.path_history is None:
            self.dragging = False
            self.start_x = None
            self.start_y = None
            self.curr_cell = (-1, -1)
            
            # Bind mouse events
            self.canvas.bind("<ButtonPress-1>", self.start_drag)
            self.canvas.bind("<B1-Motion>", self.on_drag)
            self.canvas.bind("<ButtonRelease-1>", self.stop_drag)

            # Add print button
            self.print_button = tk.Button(
                self.root,
                text="Run Simulation",
                command=self.button_press
            )
            self.print_button.pack(pady=5)

        if self.path_history:
            # Start the animation loop
            self.animate_path()

    def animate_path(self):
        
        
        if self.path_history and self.current_path_index < len(self.path_history):
            current_path = self.path_history[self.current_path_index]
            
            if self.current_coord_index < len(current_path):
                # Get the current coordinate and update the grid
                coord = current_path[self.current_coord_index]
                self.grid[coord[0]][coord[1]] = 10
                #self.draw_grid()
                
                # Move to next coordinate
                self.current_coord_index += 1
                
                # Schedule the next animation step
                self.root.after(0, self.animate_path)
            else:
                # Move to next path
                self.current_path_index += 1
                self.current_coord_index = 0
                self.draw_grid()
                if self.current_path_index < len(self.path_history): self.grid = np.copy(self.grid_copy)
                
                self.root.after(500, self.animate_path)

    def draw_grid(self):
        self.canvas.delete("all")

        for row in range(self.rows):
            y = row * self.cell_size + self.cell_size/2 + self.number_padding
            self.canvas.create_text(
                self.number_padding/2, 
                y,
                text=str(row),
                font=('Arial', 10)
            )

        # Draw column numbers
        for col in range(self.cols):
            x = col * self.cell_size + self.cell_size/2 + self.number_padding
            self.canvas.create_text(
                x,
                self.number_padding/2,
                text=str(col),
                font=('Arial', 10)
            )
        
        # Fill cells based on array values
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid[row][col] == 10:
                    self.fill_cell(row, col, 'blue')
                elif self.grid[row][col] == 1:  # Path cells
                    self.fill_cell(row, col, 'gray')
        
        # start cell
        if self.start is not None: self.fill_cell(self.start[0],self.start[1], 'green')

        # end cell
        if self.end is not None: self.fill_cell(self.end[0],self.end[1], 'red')

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
    
    def button_press(self):
        self.root.destroy()
    
    def run(self):
        self.root.mainloop()
        return self.grid


class AStarDebug(Node):
    ctx: Context
    def __init__(self, ctx: Context) -> None:
        super().__init__("astar_debug")

        self.ctx = ctx
        self.ctx.node = self
        self.declare_parameters("",[("search.traversable_cost", Parameter.Type.DOUBLE)])
        self.set_parameters([Parameter("search.traversable_cost", Parameter.Type.DOUBLE, 0.2)])
        costmap = CostMap()
        costmap.origin = np.array([0,0])
        costmap.height = 15
        costmap.width = 10
        costmap.resolution = 1

        start = np.array([3,3])
        end = np.array([8,8])

        sim = SimWindow(costmap.height, costmap.width, 50, start=start, end=end)
        
        costmap_data = sim.run()
        costmap.data = costmap_data

        self.ctx.env = Environment(self.ctx, image_targets=ImageTargetsStore(self.ctx), cost_map=costmap)

        astar = AStar(np.array([0,0]), self.ctx)

        path_history = astar.a_star(start, end, debug=True)

        sim = SimWindow(costmap.height, costmap.width, 50, grid=costmap_data, path_history=path_history, start=start, end=end)
        sim.run()
        
        for path in path_history or []:
            temp = ""
            self.get_logger().info(str(path))

                



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
