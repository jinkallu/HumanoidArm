import numpy as np
from dm_control import mujoco
from dm_control.rl import control
from dm_control.rl.control import specs
import dm_env


from hand import Hand

class MyCustomEnv(control.Environment):
    def __init__(self, physics, task, time_limit=20):
        #physics = mujoco.Physics.from_xml_path(xml_path)
        self.hand = Hand()
        super().__init__(physics, task, time_limit)

    # def action_spec(self):
    #     # Define the action space, e.g., a single actuator with control range [-1, 1]
    #     return specs.BoundedArray(
    #         shape=(1,), dtype=np.float32, minimum=-1, maximum=1
    #     )

    # def observation_spec(self):
    #     # Define the observation space, e.g., joint position and velocity
    #     return {
    #         'position': specs.Array(shape=(1,), dtype=np.float32),
    #         'velocity': specs.Array(shape=(1,), dtype=np.float32)
    #     }

    # def get_observation(self):
    #     # Return the observation (e.g., joint position and velocity)
    #     return {
    #         'position': self.physics.named.data.qpos,
    #         'velocity': self.physics.named.data.qvel
    #     }

    # def _step(self, action):
    #     # Apply action (control the motor)
    #     self.physics.named.data.ctrl['hinge'] = action
    #     self.physics.step()

    #     # Return observation, reward, and termination signal
    #     observation = self.get_observation()
    #     reward = -np.abs(observation['position'])  # Example reward: minimize position
    #     return control.Environment.TimeStep(
    #         step_type=control.StepType.MID,
    #         reward=reward,
    #         discount=1.0,
    #         observation=observation
    #     )
    
    def madel_name_to_index(self, name, type):
        """Retrieve the index of a model item by its name."""
        try:
            index = self.physics.model.name2id(name, type)
            return index
        except ValueError:
            print(f"model item '{name}' not found!")
            return None
    
    def get_joint_names(self):
        """Retrieve all joint names from the model."""
        joint_names = [self.physics.model.id2name(i, 'joint') for i in range(self.physics.model.njnt)]
        return joint_names
    
    def set_initial_joint_positions(self, positions):
        qpos = self.physics.data.qpos
        for position in positions:
            if position["type"] == "joint":
                joint_name = position["name"]
                joint_position = position["pos"]
                joint_index = self.madel_name_to_index(joint_name, "actuator")
                #qpos[joint_index] = joint_position
                self.physics.data.ctrl[joint_index] = joint_position
            elif position["type"] == "tendon":
                tendon_name = position["name"]
                tendon_position = position["pos"]
                actuator_index = self.madel_name_to_index(tendon_name, 'actuator')
                self.physics.data.ctrl[actuator_index] = tendon_position
                

    # def get_hand_relaxed_pose(self):
    #     initial_positions = [
    #         {"type": "joint", "name": "rh_A_LFJ5", "pos": 0.0}, # Little Finger ctrlrange="0 0.785398" Towards thumb
    #         {"type": "joint", "name": "rh_A_LFJ4", "pos": 0.0}, # Little Finger ctrlrange="-0.349066 0.349066" sideways
    #         {"type": "joint", "name": "rh_A_LFJ3", "pos": 1.5}, # Little Finger ctrlrange="-0.261799 1.5708" Flexing MCP
    #         {"type": "tendon", "name": "rh_A_LFJ0", "pos": 3.14} # Little Finger, flexing DIP and PIP 
    #     ]
    #     return initial_positions        
    
    # def reset(self):
    #     #initial_positions = self.get_hand_relaxed_pose()
    #     # flexed_finger = self.hand.touch()
    #     # self.set_initial_joint_positions(flexed_finger)
    #     with self._physics.reset_context():
    #         self._task.initialize_episode(self._physics)
    #     observation = self._task.get_observation(self._physics)
    #     t_step =  dm_env.TimeStep(
    #         step_type=dm_env.StepType.FIRST,
    #         reward=None,
    #         discount=None,
    #         observation=observation)
    #     print(t_step)
    #     return t_step
        
        

    

