#!/usr/bin/env python
"""
Panda UI Controller.
Uses 7 sliders to manipulate the position of the panda
"""
import numpy as np

from kivy.app import App
from kivy.uix.slider import Slider
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label

from hiro_ros_arm_controller.RobotController import PandaController


class PandaPositionMover(GridLayout):
    def __init__(self, **kwargs):
        super(PandaPositionMover, self).__init__(**kwargs)
        self.controller = PandaController()
        # Start from default value in sliders
        self.desired_positions = [0, 0, 0, -0.0698, 0, 0, 0]
        # set panda to initial positions
        self.controller.publish_positions(self.desired_positions, 0.1)
        self.cols = 3
        # The limits are from the website:
        # https://frankaemika.github.io/docs/control_parameters.html
        self.label_values = []
        limits = np.array([[-2.8973, 2.8973],
                           [-1.7628, 1.7628],
                           [-2.8973, 2.8973],
                           [-3.0718, -0.0698],
                           [-2.8973, 2.8973],
                           [-0.0175, 3.7525],
                           [-2.8973, 2.8973]])
        for idx in range(7):
            joint_name = "Joint {}".format(idx+1)
            starting_pos = self.desired_positions[idx]
            slider = Slider(min=float(limits[idx, 0]), max=float(limits[idx, 1]), step=0.001, value=starting_pos, id=joint_name)
            self.add_widget(Label(text="Panda {}".format(joint_name)))
            self.add_widget(slider)
            label = Label(text=str(starting_pos))
            self.label_values.append(label)
            self.add_widget(label)
            slider.bind(value=self.slider_value_change)

    def slider_value_change(self, instance, joint_value):
        joint_idx = int(instance.id[-1]) - 1
        self.label_values[joint_idx].text = str(joint_value)
        self.desired_positions[joint_idx] = joint_value
        self.controller.publish_positions(self.desired_positions, 0.1)


class PandaSlider(App):
    def build(self):
        widgetcontainer = PandaPositionMover()
        return widgetcontainer


if __name__ == "__main__":
    PandaSlider().run()
