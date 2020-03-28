#!/usr/bin/env python
"""
Panda UI Controller.
To use this please install the dependencies:
sudo pip install kivy
sudo pip install pygame
"""
from kivy.app import App
from kivy.uix.slider import Slider
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
import sys
import rospkg
sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts.controllers.PandaController import PandaController  # noqa: E402


class panda_position_mover(GridLayout):
    def __init__(self, **kwargs):
        super(panda_position_mover, self).__init__(**kwargs)
        self.controller = PandaController()
        # Start from default value in sliders
        self.controller.publish_positions([0.0, 0.0, 0.0, -0.0698, 0.0, 0.0, 0.0], 0.1)
        # GUI starting
        self.cols = 3
        # The limits are from the website:
        # https://frankaemika.github.io/docs/control_parameters.html
        my_slider1 = Slider(min=-2.8973, max=2.8973, step=0.001, value=0, id="Joint 1")
        my_slider2 = Slider(min=-1.7628, max=1.7628, step=0.001, value=0, id="Joint 2")
        my_slider3 = Slider(min=-2.8973, max=2.8973, step=0.001, value=0, id="Joint 3")
        my_slider4 = Slider(min=-3.0718, max=-0.0698, step=0.001, value=-0.0698, id="Joint 4")
        my_slider5 = Slider(min=-2.8973, max=2.8973, step=0.001, value=0, id="Joint 5")
        my_slider6 = Slider(min=-0.0175, max=3.7525, step=0.001, value=0, id="Joint 6")
        my_slider7 = Slider(min=-2.8973, max=2.8973, step=0.001, value=0, id="Joint 7")

        # I know this can be upgraded but I just wrote
        # Joint 1
        self.add_widget(Label(text='Panda Joint 1'))
        self.add_widget(my_slider1)
        self.joint1_value = Label(text='0')
        self.add_widget(self.joint1_value)
        my_slider1.bind(value=self.slider_value_change)

        # Joint 2
        self.add_widget(Label(text='Panda Joint 2'))
        self.add_widget(my_slider2)
        self.joint2_value = Label(text='0')
        self.add_widget(self.joint2_value)
        my_slider2.bind(value=self.slider_value_change)

        # Joint 3
        self.add_widget(Label(text='Panda Joint 3'))
        self.add_widget(my_slider3)
        self.joint3_value = Label(text='0')
        self.add_widget(self.joint3_value)
        my_slider3.bind(value=self.slider_value_change)

        # Joint 4
        self.add_widget(Label(text='Panda Joint 4'))
        self.add_widget(my_slider4)
        self.joint4_value = Label(text='-0.0698')
        self.add_widget(self.joint4_value)
        my_slider4.bind(value=self.slider_value_change)

        # Joint 5
        self.add_widget(Label(text='Panda Joint 5'))
        self.add_widget(my_slider5)
        self.joint5_value = Label(text='0')
        self.add_widget(self.joint5_value)
        my_slider5.bind(value=self.slider_value_change)

        # Joint 6
        self.add_widget(Label(text='Panda Joint 6'))
        self.add_widget(my_slider6)
        self.joint6_value = Label(text='0')
        self.add_widget(self.joint6_value)
        my_slider6.bind(value=self.slider_value_change)

        # Joint 7
        self.add_widget(Label(text='Panda Joint 7'))
        self.add_widget(my_slider7)
        self.joint7_value = Label(text='0')
        self.add_widget(self.joint7_value)
        my_slider7.bind(value=self.slider_value_change)

    def slider_value_change(self, instance, joint_value):
        if instance.id == "Joint 1":
            self.joint1_value.text = "%f" % joint_value
        elif instance.id == "Joint 2":
            self.joint2_value.text = "%f" % joint_value
        elif instance.id == "Joint 3":
            self.joint3_value.text = "%f" % joint_value
        elif instance.id == "Joint 4":
            self.joint4_value.text = "%f" % joint_value
        elif instance.id == "Joint 5":
            self.joint5_value.text = "%f" % joint_value
        elif instance.id == "Joint 6":
            self.joint6_value.text = "%f" % joint_value
        elif instance.id == "Joint 7":
            self.joint7_value.text = "%f" % joint_value
        positions = [float(self.joint1_value.text),
                     float(self.joint2_value.text),
                     float(self.joint3_value.text),
                     float(self.joint4_value.text),
                     float(self.joint5_value.text),
                     float(self.joint6_value.text),
                     float(self.joint7_value.text)
                     ]
        self.controller.publish_positions(positions, 0.1)


class PandaSlider(App):
    def build(self):
        widgetcontainer = panda_position_mover()
        return widgetcontainer


if __name__ == "__main__":
    PandaSlider().run()
