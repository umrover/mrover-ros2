#!/usr/bin/env python3

from __future__ import annotations
import signal
import graphviz  # type: ignore
import time
from PyQt5.QtWidgets import *  # type: ignore
from PyQt5.QtCore import *  # type: ignore
from PyQt5.QtGui import QPainter  # type: ignore
from PyQt5.QtSvg import QSvgRenderer  # type:ignore

import rclpy
import rclpy.time
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import sys
from mrover.msg import StateMachineStructure, StateMachineStateUpdate
from threading import Lock
from dataclasses import dataclass
from typing import Optional, List, Dict
import threading


STRUCTURE_TOPIC = "nav_structure"
STATUS_TOPIC = "nav_state"


@dataclass
class State:
    name: str
    children: List[State]


class StateMachine:
    def __init__(self):
        self.states: Dict[str, State] = {}
        self.structure: Optional[StateMachineStructure] = None
        self.mutex: Lock = Lock()
        self.cur_active: str = ""
        self.previous_state: str = ""
        self.needs_redraw: bool = True

    def set_active_state(self, active_state):
        """
        sets the state specified to be active (thread safe)
        """
        with self.mutex:
            if active_state != self.cur_active and active_state in self.states:
                self.previous_state = self.cur_active
                self.cur_active = active_state
                self.needs_redraw = True
                now = rclpy.time.Time()
                rclpy.logging.get_logger("Visualizer").info(
                    f"Current time: {now} Previous state: {self.previous_state} Current State: { self.cur_active}"
                )

    def _rebuild(self, structure: StateMachineStructure):
        """
        rebuilds the state dictionary with a new structure message
        """
        self.states = {child.origin: State(child.origin, []) for child in structure.transitions}
        for transition in structure.transitions:
            origin = transition.origin
            for to in transition.destinations:
                self.states[origin].children.append(self.states[to])
        self.needs_redraw = True

    def check_rebuild(self, structure: StateMachineStructure):
        """
        checks if the structure passed as input matches the structure already represented (thread safe)
        """
        with self.mutex:
            if structure == self.structure:
                return False
            else:
                self._rebuild(structure)
                self.structure = structure

    def container_status_callback(self, status: StateMachineStateUpdate):
        self.set_active_state(status.state)

    def container_structure_callback(self, structure: StateMachineStructure):
        self.check_rebuild(structure)


class GUI(QWidget):  # type: ignore
    def __init__(self, state_machine_instance, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.label: QLabel = QLabel()  # type: ignore
        self.timer: QTimer = QTimer()  # type: ignore
        self.renderer: QSvgRenderer = QSvgRenderer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1)
        self.graph: Optional[graphviz.Digraph] = None
        self.img = None
        self.state_machine: StateMachine = state_machine_instance
        self.viz = Node("Visualizer")

        self.viz.create_subscription(
            StateMachineStructure, STRUCTURE_TOPIC, self.state_machine.container_structure_callback, 1
        )

        self.viz.create_subscription(
            StateMachineStateUpdate, STATUS_TOPIC, self.state_machine.container_status_callback, 1
        )

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.img is not None:
            self.renderer.load(self.img)
        self.resize(self.renderer.defaultSize())
        self.renderer.render(painter)

    def update(self):
        rclpy.spin_once(self.viz)
        with self.state_machine.mutex:
            if self.graph is None or self.state_machine.needs_redraw:
                self.graph = graphviz.Digraph(comment="State Machine Diagram", format="svg")
                for state_name in self.state_machine.states.keys():
                    color = "red" if self.state_machine.cur_active == state_name else "black"
                    self.graph.node(state_name, color=color)

                for state in self.state_machine.states.values():
                    for child in state.children:
                        self.graph.edge(state.name, child.name)

                self.state_machine.needs_redraw = False

        self.img = self.graph.pipe()
        self.repaint()


def main():
    try:
        rclpy.init()

        signal.signal(signal.SIGINT, signal.SIG_DFL)
        state_machine = StateMachine()

        print("Node Created...")

        print("Subscriptions Created...")

        print("Node Created...")

        print("Subscriptions Created...")

        app = QApplication([])  # type: ignore
        g = GUI(state_machine)
        g.show()
        app.exec_()

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
