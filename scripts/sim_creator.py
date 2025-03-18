import tkinter as tk


def create_60x60_grid():
    root = tk.Tk()
    root.title("30×30 Grid with Toggleable Colors (including black)")

    # -------------------
    #   CONFIGURATION
    # -------------------
    GRID_SIZE = 35  # 30 cells in each dimension
    CELL_SIZE = 25  # Each cell is 30×30 pixels
    MARGIN = 30  # Extra space around the grid for the border
    TOTAL_PIXELS = GRID_SIZE * CELL_SIZE

    # Now includes "black" to represent the rover's position
    color_modes = ["white", "green", "blue", "red"]
    current_mode_index = 0  # Start with mode = "white"

    # 2D array to keep track of each cell's color (initially "white")
    grid_colors = [["white" for _ in range(GRID_SIZE)] for __ in range(GRID_SIZE)]

    # Include black → 4
    color_key = {"white": 0, "green": 1, "blue": 2, "red": 3}

    # -------------------
    #   TK WIDGETS
    # -------------------
    main_frame = tk.Frame(root)
    main_frame.pack()

    # Increase the Canvas size to accommodate margins
    CANVAS_SIZE = TOTAL_PIXELS + 2 * MARGIN

    # Left: Canvas for the 30×30 grid
    canvas = tk.Canvas(main_frame, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
    canvas.pack(side=tk.LEFT)

    # Right: A small canvas (icon) to show current mode color
    icon_size = 50
    mode_canvas = tk.Canvas(main_frame, width=icon_size, height=icon_size, bg="white")
    mode_canvas.pack(side=tk.RIGHT, padx=10)

    # Draw a rectangle showing the current mode color
    mode_rect = mode_canvas.create_rectangle(
        0, 0, icon_size, icon_size, fill=color_modes[current_mode_index], outline="black"
    )

    def update_mode_indicator():
        """Update the color of the 'mode' icon."""
        mode_canvas.itemconfig(mode_rect, fill=color_modes[current_mode_index])

    # -------------------
    #   CREATE THE GRID
    # -------------------
    rect_ids = {}  # dict: (row, col) -> rectangle_id

    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            x1 = MARGIN + col * CELL_SIZE
            y1 = MARGIN + row * CELL_SIZE
            x2 = x1 + CELL_SIZE
            y2 = y1 + CELL_SIZE

            rect_id = canvas.create_rectangle(x1, y1, x2, y2, fill="white", outline="black")
            rect_ids[(row, col)] = rect_id

    # -------------------
    #   EVENT HANDLERS
    # -------------------
    def on_click(event):
        """
        When the user clicks on a cell, toggle it between white
        and the current mode color.
        """
        x, y = event.x, event.y
        # Check if the click is inside the grid
        if not (MARGIN <= x < MARGIN + GRID_SIZE * CELL_SIZE and MARGIN <= y < MARGIN + GRID_SIZE * CELL_SIZE):
            return

        col_clicked = (x - MARGIN) // CELL_SIZE
        row_clicked = (y - MARGIN) // CELL_SIZE

        rect_id = rect_ids[(row_clicked, col_clicked)]
        current_fill = grid_colors[row_clicked][col_clicked]
        desired_fill = color_modes[current_mode_index]

        # Toggle logic: if it's already the current mode color, revert to white
        # otherwise set it to current mode color
        if current_fill == desired_fill:
            new_fill = "white"
        else:
            new_fill = desired_fill

        # Update
        canvas.itemconfig(rect_id, fill=new_fill)
        grid_colors[row_clicked][col_clicked] = new_fill

    canvas.bind("<Button-1>", on_click)

    def cycle_mode(event):
        """
        Pressing SPACE cycles to the next color mode.
        """
        nonlocal current_mode_index
        current_mode_index = (current_mode_index + 1) % len(color_modes)
        update_mode_indicator()

    # -------------------
    #   HOVER COORDINATES
    # -------------------
    hover_label = tk.Label(root, text="Hovering at: (N/A, N/A)")
    hover_label.pack()

    def on_hover(event):
        """
        Update the label with the current grid cell coordinates where the mouse is hovering,
        using (0, 0) as the center of the grid and aligning coordinates with Cartesian system.
        """
        x, y = event.x, event.y
        if MARGIN <= x < MARGIN + GRID_SIZE * CELL_SIZE and MARGIN <= y < MARGIN + GRID_SIZE * CELL_SIZE:
            col_hovered = (x - MARGIN) // CELL_SIZE
            row_hovered = (y - MARGIN) // CELL_SIZE

            # Convert to Cartesian-like coordinates
            center_offset = GRID_SIZE // 2
            adjusted_x = col_hovered - center_offset  # X-coordinate
            adjusted_y = center_offset - row_hovered  # Y-coordinate

            hover_label.config(text=f"Hovering at: ({adjusted_x}, {adjusted_y})")
        else:
            hover_label.config(text="Hovering at: (N/A, N/A)")

    canvas.bind("<Motion>", on_hover)

    def on_q_press(event):
        """
        Pressing 'q' prints the 2D array (using color_key), then creates new_sim.yaml,
        and closes the window.
        """
        # 1) Print the 2D array
        numeric_grid = []
        for r in range(GRID_SIZE):
            numeric_row = [color_key[color] for color in grid_colors[r]]
            numeric_grid.append(numeric_row)
            print(numeric_row)

        # 2) Prepare YAML header
        yaml_header = """# All units are in SI
# ===================
# Time:     second, hz
# Angle:    radian
# Distance: meter

simulator:
  ros__parameters:
    save_rate: 1.0
    save_history: 4096
    headless: false
    motor_timeout: 100

    ref_heading: 90.0  # For the GPS sensor to work

    objects:
      rover:
        type: urdf
        uri: package://mrover/urdf/rover/rover.urdf.xacro
        position: [ 0.0, 0.0, 0.1 ]
      world:
        type: urdf
        uri: package://mrover/urdf/world/world.urdf.xacro
      bottle:
        type: urdf
        uri: package://mrover/urdf/world/bottle.urdf.xacro
        position: [9.0, 10.0, 0.5]

"""

        # 3) Generate objects for rocks, ignoring cells with value 0 (white or no rock).
        rock_counter = 1
        yaml_rocks = []

        # URIs by value: 1 => small, 2 => medium, 3 => large
        uri_map = {
            1: "package://mrover/urdf/world/small_rock.urdf.xacro",
            2: "package://mrover/urdf/world/medium_rock.urdf.xacro",
            3: "package://mrover/urdf/world/large_rock.urdf.xacro",
        }
        # For demonstration, different z's by size
        z_map = {1: "0.5", 2: "1.0", 3: "1.0"}

        center_offset = GRID_SIZE // 2  # Center offset for coordinate transformation

        for row_i in range(GRID_SIZE):
            for col_i in range(GRID_SIZE):
                val = numeric_grid[row_i][col_i]
                # Only create a rock if val in {1,2,3}
                if val in uri_map:
                    # Adjust coordinates for the YAML output
                    x = col_i - center_offset  # X-coordinate
                    y = center_offset - row_i  # Y-coordinate (invert Y-axis)
                    rock_name = f"rock_{rock_counter}"
                    rock_counter += 1

                    lines = [
                        f"      {rock_name}:",
                        f"        type: urdf",
                        f"        uri: {uri_map[val]}",
                        f"        position: [ {x:.2f}, {y:.2f}, {z_map[val]} ]\n",
                    ]
                    yaml_rocks.append("\n".join(lines))

        # 4) Add finishing lines
        yaml_footer = """    ref_lat: 38.4225202
    ref_lon: -110.7844653
    ref_alt: 0.0
    world_frame: "map"
    rover_frame: "sim_base_link"
"""

        # 5) Write out to new_sim.yaml
        with open("config/simulator.yaml", "w") as f:
            f.write(yaml_header)
            if yaml_rocks:
                f.write("      # Auto-generated rocks from the grid\n")
                f.write("\n".join(yaml_rocks))
                f.write("\n")
            f.write(yaml_footer)

        print("YAML file successfully written to config/simulator.yaml.")
        # Close the window
        root.destroy()

    # Bind events
    root.bind("<space>", cycle_mode)
    root.bind("q", on_q_press)

    # -------------------
    #   LABEL IN CENTER
    # -------------------
    center = GRID_SIZE // 2
    center_x = MARGIN + center * CELL_SIZE + (CELL_SIZE // 2)
    center_y = MARGIN + center * CELL_SIZE + (CELL_SIZE // 2)
    canvas.create_text(center_x, center_y, text="(0,0)", fill="black", anchor="center")

    # Show the initial mode color in the icon
    update_mode_indicator()
    root.mainloop()


if __name__ == "__main__":
    create_60x60_grid()