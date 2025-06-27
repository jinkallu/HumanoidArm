# HumanoidArm: Simulating Human-Like Robotic Grasping in MuJoCo

## Overview

HumanoidArm is an experimental research project that simulates a "robotic brain"—an intelligent system that can perceive, plan, and manipulate objects in a 3D environment, inspired by human hand-eye coordination and tactile feedback. The simulation uses a Shadow Hand robot model in the MuJoCo physics engine, computer vision for object detection, and point cloud completion to understand 3D object shapes, culminating in motion planning and grasping that mimics human strategies.

> _"This project is my attempt to mimic how humans intuitively reach out and grasp objects—predicting their shape, planning a path, and adjusting grip using sight and touch."_

## Motivation

Inspired by the layered complexity of human hand movements—seeing, planning, and grasping—the goal is to explore how vision, memory, motion planning, and tactile feedback can be integrated in a robotics simulation. The project is part of a personal series exploring challenging, unfinished (but valuable) robotics ideas.

---

## Key Features

- **Simulated Shadow Hand (E3M5):** Uses highly-detailed models of the Shadow Hand robot (right and left handed), with assets converted from URDF to MJCF for MuJoCo.
- **Perception Pipeline:**
  - **RGB + Depth Sensing:** Leverages MuJoCo's cameras to obtain both RGB and depth images of the scene.
  - **Object Detection:** Applies [Detectron2](https://github.com/facebookresearch/detectron2) to segment and identify objects (e.g., a ball) in the RGB image.
  - **3D Shape Completion:** Maps the segmented region onto the depth image; converts the result to a point cloud, and uses a Point Completion Network (PCN) to infer the full 3D geometry from partial observations.
- **Robot State Awareness:** The robot has proprioceptive feedback (knows joint/finger positions) and exteroceptive sensing (touch/pressure).
- **Motion Planning:**
  - **Goal:** Reach and grasp the object using minimal energy, while avoiding obstacles.
  - **Algorithms:** Uses Rapidly-exploring Random Trees (RRT) with energy minimization, and fallback to ray tracing.
  - **Continuous Replanning:** At every step, the robot updates its path using new visual and sensor data.
  - **Inverse Kinematics:** Computes joint and actuator targets for each incremental movement.
- **Grasp Execution:**
  - As the hand approaches the object, the control loop switches focus from vision to tactile feedback, using simulated pressure sensors for grip adjustment.
  - The grasp becomes more precise as the robot nears the target.

---

## Code Architecture

- **Simulation Core:** Built on [dm_control](https://github.com/deepmind/dm_control) and MuJoCo physics, with custom environments for the Shadow Hand.
- **Perception Modules:**
  - `my_detectron2.py`: Detectron2 wrapper for object segmentation.
  - `pc_predictor.py` & `pcn_model/pcn.py`: Point Completion Network for 3D shape prediction.
- **Motion & Control:**
  - `arm.py`: Main pipeline integrating perception, planning, and control. Implements object detection, 3D reconstruction, path planning, and actuation.
  - `rest_task.py`: Task logic, including grasping routines and inverse kinematics via `inv_kin.py`.
  - `hand.py`: High-level hand/finger posture management.
- **Assets:** The `shadow_hand/` directory contains MJCF models, meshes, textures, and conversion details.

---

## Example Pipeline

1. **See:** The robot receives an RGB and depth image from MuJoCo simulation.
2. **Detect:** Detectron2 identifies and segments objects of interest (e.g., a ball).
3. **Reconstruct:** The masked region is mapped to a partial point cloud, then PCN predicts the full 3D shape.
4. **Plan:** The robot plans its approach path using RRT, minimizing energy and avoiding obstacles.
5. **Move:** Inverse kinematics computes the joint positions for each step.
6. **Grasp:** As the hand nears the object, the system emphasizes tactile feedback, adjusting grip force in real time.

---

## Technologies Used

- **Simulation:** [MuJoCo](https://mujoco.org/) 2.2.2+, [dm_control](https://github.com/deepmind/dm_control)
- **Perception:** [Detectron2](https://github.com/facebookresearch/detectron2), OpenCV, Matplotlib, Open3D
- **3D Completion:** Point Completion Network (PCN, PyTorch)
- **Planning:** RRT, Inverse Kinematics (custom, based on MuJoCo)
- **Robotic Model:** Shadow Hand E3M5 (see `shadow_hand/README.md` for details and license)

---

## Getting Started

> **Note:** This is an advanced research prototype. Some modules are experimental, and the project is not complete.

1. **Install requirements:**  
   - MuJoCo 2.2.2+
   - Python libraries: `dm_control`, `detectron2`, `torch`, `opencv-python`, `open3d`, `matplotlib`
2. **Prepare assets:**  
   - Download/convert Shadow Hand MJCF files as described in `shadow_hand/README.md`.
   - Provide trained weights for Detectron2 and PCN.
3. **Run the simulation:**  
   - Edit and execute `arm.py` to start the perception-to-grasp pipeline.

---

## Lessons Learned

- Even simple human-like grasping involves complex, layered perception, memory, planning, and feedback.
- Integrating vision, 3D reconstruction, motion planning, and tactile sensing is challenging but essential for realistic robotic manipulation.
- Real-time adaptation—updating plans as new data arrives—mirrors how biological systems work.

> _"This project helped me appreciate how hard it is to replicate what the brain does effortlessly. Even unfinished, it was a valuable learning experience."_

---

## License

- The Shadow Hand models are released under the [Apache-2.0 License](shadow_hand/LICENSE).
- Code in this repository is intended for research and educational use.

---

## References

- [Shadow Robot Company](https://www.shadowrobot.com/)
- [Detectron2](https://github.com/facebookresearch/detectron2)
- [PCN: Point Completion Network](https://arxiv.org/abs/1808.00671)
- [MuJoCo](https://mujoco.org/)
- [dm_control](https://github.com/deepmind/dm_control)

---
