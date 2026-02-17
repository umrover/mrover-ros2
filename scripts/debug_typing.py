#!/usr/bin/env python3

import sys
import rclpy
from mrover.action import TypingPosition
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.client import Client
from rclpy.task import Future
from rclpy.time import Time
from rclpy.duration import Duration
import tkinter as tk
from mrover.srv import IkMode
import time

class DebugTyping(Node):
    root: tk.Tk
    ik_mode_client: Client
    typing_client: ActionClient
    goal_entries: list[tk.Entry]
    typing_future: Future | None
    cancel: bool
    
    def __init__(self, root: tk.Tk):
        super().__init__("debug_typing")
        self.root = root
        self.cancel = False
        self.current_goal_handle = None
        self.typing_future = None

        self.ik_mode_client = self.create_client(IkMode, "ik_mode")
        self.typing_client = ActionClient(self, TypingPosition, "typing_ik")

        entry_label = tk.Label(root, text="Typing Coordinates [x, y]")
        entry_label.pack()

        self.goal_entries = []
        NUM_ENTRIES = 10
        for i in range(NUM_ENTRIES):
            entry_box = tk.Entry(root)
            entry_box.pack()
            self.goal_entries.append(entry_box)

        self.root.after(10, self.update_tk_callback)

        def set_typing():
            self.ik_mode_client.call_async(IkMode.Request(mode=2))

        btn_frame = tk.Frame(root)
        btn_frame.pack(fill="x", pady=5)

        set_typing_btn = tk.Button(btn_frame, text="Set Typing Origin", command=set_typing)
        set_typing_btn.pack(side="left", padx=5)

        send_goals_button = tk.Button(btn_frame, text="Send Goals", command=self.send_goals)
        send_goals_button.pack(side="left", padx=5)
        
        cancel_goals_button = tk.Button(btn_frame, text="Cancel Goals", command=self.cancel_goals)
        cancel_goals_button.pack(side="right", padx=5)

    def update_tk_callback(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.update_idletasks()
        self.root.after(10, self.update_tk_callback)

    def send_goals(self):
        self.cancel = False
        for entry in self.goal_entries:
            val = entry.get().strip()
            if not val:
                continue

            entry_input = val.split(" ")
            goal = TypingPosition.Goal(x=float(entry_input[0]), y=float(entry_input[1]))
            self.get_logger().info(f"Sending goal {goal}")

            self.typing_future = self.typing_client.send_goal_async(goal)
            
            while not self.typing_future.done():
                rclpy.spin_once(self, timeout_sec=0)
                self.root.update()
                if self.cancel: return
            
            self.current_goal_handle = self.typing_future.result()

            if not self.current_goal_handle.accepted:
                self.get_logger().info('Goal rejected')
                continue

            self.get_logger().info('Goal accepted')

            result_future = self.current_goal_handle.get_result_async()
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0)
                self.root.update()
                if self.cancel: 
                    return
            
            finish_time = self.get_clock().now()
            while((self.get_clock().now() - finish_time) < Duration(seconds=1.5)):
                rclpy.spin_once(self, timeout_sec=0)
                self.root.update()

            self.get_logger().info('Goal finished')

    def cancel_goals(self):
        self.cancel = True
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling goal...')
            self.current_goal_handle.cancel_goal_async()

if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    root = tk.Tk()
    root.title("Debug Typing")
    node = DebugTyping(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.shutdown()