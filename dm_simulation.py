# The basic mujoco wrapper.
from dm_control import mujoco


# Access to enums and MuJoCo library functions.
from dm_control.mujoco.wrapper.mjbindings import enums
from dm_control.mujoco.wrapper.mjbindings import mjlib

import matplotlib.pyplot as plt


class DMSimulation():
    "Simulation based of Deepmind DM Control"
    def __init__(self, xml_path):
        self.xml_path = xml_path
        self.physics = mujoco.Physics.from_xml_path(xml_path)

    def get_physics(self):
        return self.physics
    

    def display_depthImg(self, depth_img):
        plt.imshow(depth_img)
        plt.axis('off')  # Turn off axis labels
        plt.show()

    def display_rgb(self, rgb):
        plt.imshow(rgb)
        plt.axis('off')  # Turn off axis labels
        plt.show()


    
