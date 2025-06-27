from dm_control.suite import base
from dm_control import mujoco

import collections
import numpy as np
from dm_env import specs
from hand import Hand

import inv_kin
from scipy.spatial.transform import Rotation as R




class RestTask(base.Task):
  """A CarShadow Hand `Task` to relax the hand to resting position.

  State is initialized in a relaxed hand state
  configuration.
  """
  def __init__(self, rest = True, coms=[]):
    self.rest = rest
    self.coms = coms

    self.grabb_started = False
    self.site_quat = None

    self.grabbed = False
    self.ff_joint_names = None
    self.ff_joint_pos = None
    self.th_joint_names = None
    self.th_joint_pos = None

    self.grab_with_th_ff_ray_th = None
    self.grab_with_th_ff_ray_ff = None

    self.i = 0
    self.th_touch_point = None


    self.hand = Hand()
    super().__init__()

  def model_name_to_index(self, name, type, physics):
    """Retrieve the index of a model item by its name."""
    try:
        index = physics.model.name2id(name, type)
        return index
    except ValueError:
        print(f"model item '{name}' not found!")
        return None

  def set_actuator_controls(self, positions, physics):
    for position in positions:
        if position["type"] == "joint":
            joint_name = position["name"]
            joint_position = position["pos"]
            joint_index = self.model_name_to_index(joint_name, "actuator", physics)
            #qpos[joint_index] = joint_position
            physics.data.ctrl[joint_index] = joint_position
        elif position["type"] == "tendon":
            tendon_name = position["name"]
            tendon_position = position["pos"]
            actuator_index = self.model_name_to_index(tendon_name, 'actuator', physics)
            physics.data.ctrl[actuator_index] = tendon_position

  def set_actions(self, positions, action, physics):
    for position in positions:
        if position["type"] == "joint":
            joint_name = position["name"]
            joint_position = position["pos"]
            joint_index = self.model_name_to_index(joint_name, "actuator", physics)
            #qpos[joint_index] = joint_position
            action[joint_index] = joint_position
        elif position["type"] == "tendon":
            tendon_name = position["name"]
            tendon_position = position["pos"]
            actuator_index = self.model_name_to_index(tendon_name, 'actuator', physics)
            action[actuator_index] = tendon_position

  def raycast(self, physics, start, end, body_id_to_exclude = -1):
     # Define the start and end points for the ray (in world coordinates)

    ray_direction = end - start
    ray_direction_normalized = ray_direction / np.linalg.norm(ray_direction)

    print("direction ", end - start, "norm ", ray_direction_normalized)


    # ray_start = np.array(start, dtype=np.float64).reshape(3, 1)
    # ray_end = np.array(end, dtype=np.float64).reshape(3, 1)
    # ray_dir = ray_end - ray_start
    # ray_dir_norm = ray_dir / np.linalg.norm(ray_dir)

    #print(ray_start, ray_end, ray_dir)

    # Initialize the ray hit data structure

    geomid = np.array([-1], dtype=np.int32)

    
    # Perform raycasting using mujoco.mj_ray()
    result = mujoco.mj_ray(
        physics.model.ptr,                # Model pointer
        physics.data.ptr,                 # Data pointer
        start,                            # Starting point
        ray_direction_normalized,         # Ray direction (vector from start to end)
        None,                             # No specific geomgroup filter (optional)
        True,                             # include_static_geoms - if False, we exclude geoms that are children of worldbody.
        body_id_to_exclude, #body_id_to_exclude,               # Not excluding any bodies
        geomid                            # Output: stores geomid if hit
    )

    print(result)

    if result > 0:  # If result > 0, ray hit something
        print ("res ", result)
        distance = result   # This is the distance from the start of the ray to the hit point
        
        # Calculate the intersection point
        offset = 0.00 # try not to touch
        hit_point = start + (distance - offset) * ray_direction_normalized
        #print("ray start ", ray_start, "distance + ray_dir", distance * ray_direction_normalized)
        
        # Print the intersection point
        print("Bodies intersected ", len(geomid))
        for id in geomid:
          print("intersected body id ", id)
          geom_name = physics.model.id2name(id, "geom")
          print(f"Ray hit geom with ID {id} {geom_name} at position: {hit_point}")
        return hit_point
    else:
        print("No intersection detected.")

  def ik_move_to_target(self, physics, start, target_name = "test_sphere"):
    end = physics.named.data.xpos[target_name] # target position
    start = physics.named.data.xpos["rh_thd_skin_1"] # target position
    body_id_to_exclude = physics.model.name2id('rh_thd_skin_1', "body")
    hit_point = self.raycast(physics, start = start, end=end, body_id_to_exclude=body_id_to_exclude)
    if hit_point is None:
       print("")
       return
    #hit_point = np.array(hit_point, dtype=np.float64).reshape(3)
    joint_names = ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"]
    res = inv_kin.qpos_from_site_pose(physics, site_name="thd_skin_1", target_pos=hit_point, joint_names=joint_names, inplace=False)
    print(res)
    if not res.success:
       return
    
    joint_pos = []
    ik_joint_indices = []
    for joint_name in joint_names:
      joint_idx = physics.model.name2id(joint_name, "joint")
      physics.data.qpos[joint_idx] = res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0
      ik_joint_indices.append(joint_idx)
      joint_pos.append(res.qpos[joint_idx])

    # Step 3: Lock all other joints by reapplying their original positions
    for idx in range(len(physics.data.qpos)):
      if idx not in ik_joint_indices:
        physics.data.qpos[idx] = self.current_qpos[idx]  # Restore original positions for non-IK joints

    return joint_names, joint_pos
  
  def desired_site_quat(self, physics, site_name, unit_direction, axis='z'):
     # align the site direction to -unit_direction
     # Get the site ID
     site_id = physics.model.name2id(site_name, "site")
    
     # Get the site's global orientation quaternion
     orientation_quat = physics.data.xquat[site_id]
     print("Site quat", orientation_quat)
     # Convert quaternion to a rotation matrix
     rotation_matrix = R.from_quat(orientation_quat).as_matrix()
     print("Site rot matrix", rotation_matrix)

     # Choose the axis direction
     axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]
     direction = rotation_matrix[:, axis_index]  # Column of rotation matrix

     # Opposite of the given direction
     opposite_unit_direction = -unit_direction
     # Compute axis of rotation (cross product)
     axis = np.cross(direction, opposite_unit_direction)
     axis_norm = np.linalg.norm(axis)
      
     if axis_norm < 1e-6:  # If vectors are parallel or anti-parallel
          if np.dot(direction, opposite_unit_direction) < 0:  # 180-degree rotation
              # Use an arbitrary orthogonal axis
              axis = np.array([1, 0, 0])
          else:
              # No rotation needed
              return np.array([1, 0, 0, 0])  # Identity quaternion
      
     axis = axis / axis_norm  # Normalize axis
      
     # Compute angle of rotation
     cos_theta = np.dot(direction, opposite_unit_direction)
     theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))  # Clip to handle numerical precision issues
      
     # Compute quaternion
     quat_w = np.cos(theta / 2)
     quat_xyz = np.sin(theta / 2) * axis
     quat = np.concatenate(([quat_w], quat_xyz))
     print("Calculated Quat", quat)
    
     return quat
  
  def define_th_ff_grab_points(self, physics, object_name, geom_name, th_site_name):
     # this function is used to define the touch points for thumb and first finger
     # since the thum is already aligned with teh sphere, x and y axis, we use form thumb site, but z, teh center of sphere
     # Now we have a source position. create a ray that starts from this source towards the direction of center of sphere.
     # find the intersection points on the surface of sphere.
     # first intersection point is teh touch point for thumb site
     # second intersection point is the touch point for first finger.
     th_touch_point = [0, 0, 0]
     ff_touch_point = [0, 0, 0]

     current_th_site_pos = physics.named.data.xpos[th_site_name]

     object_geom_id = physics.model.name2id(geom_name, "geom")
     center = physics.data.geom_xpos[object_geom_id]
     radius = physics.model.geom_size[object_geom_id][0]

     # calculate ray intersection point on sphere object surface from the th site to the sphere center direction
     th_hit_point = self.raycast(physics, current_th_site_pos, center)

     th_touch_point = th_hit_point

     
     return th_touch_point, ff_touch_point
  
  def grab_with_th_ff_new(self, physics, object_name, geom_name):
     # This function is used to first define the grab points in sphere object and then 
     # use Thumb and first finger to grab teh object
     # provided that the hand is already aligned with the sphere object
     th_site_body_name = "rh_thd_skin_1"
     if self.th_touch_point is None:
      th_touch_point, ff_touch_point = self.define_th_ff_grab_points(physics, object_name, geom_name, th_site_body_name)
      self.th_touch_point = th_touch_point

     th_site_name = "thd_skin_1"
     th_joints = [
        "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"
     ]
     self.move_any_site_to_target(physics, self.th_touch_point, th_site_name, joints=th_joints)
     
  def move_any_site_to_target(self, physics, target_point, site_name, joints): 
    # function to move any site to a target point with given joints movements
    tol = 1e-1
    th_res = inv_kin.qpos_from_site_pose(physics, site_name=site_name, target_pos=target_point, joint_names=joints, inplace=False, rot_weight=0.5, tol=tol, max_steps=1000)
    
    if not th_res.success:
      print("Error! Could not find joint positions for th touch point", th_res)
      return
    
    all_joints_idx = []
    for joint_name in joints:
      joint_idx = physics.model.name2id(joint_name, "joint")
      all_joints_idx.append(joint_idx)
      physics.data.qpos[joint_idx] = th_res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0

    ik_joint_indices = []
    for joint_name in joints:
      joint_idx = physics.model.name2id(joint_name, "joint")
      ik_joint_indices.append(joint_idx)
      physics.data.qpos[joint_idx] = th_res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0

    # Lock all other joints by reapplying their original positions
    for idx in range(len(physics.data.qpos)):
      if idx not in ik_joint_indices and idx in all_joints_idx:
        physics.data.qpos[idx] = self.current_qpos[idx]  # Restore original positions for non-IK joints
  
  def move_site_to_target(self, physics, object_name, geom_name, site_name="palm_skin_1"):
    mean_contact_point = self.calculate_mean_contact_point(physics, object_name, geom_name) # it should called only once
    target_point, unit_direction = self.calculate_normal_through_com(physics, mean_contact_point, geom_name, scale=4.2)
    print(target_point)
    end = target_point#physics.named.data.xpos[object_name] # should be surface
    th_start = physics.named.data.xpos["rh_thd_skin_1"]

    all_joints = ["rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
                  "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
                  "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
                  "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4",
                  "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5",
                  "rh_WRJ1", "rh_WRJ2",
                  "rh_EBJ", "rh_EBJR",
                  "rh_SHJ1", "rh_SHJ2"
                  ]
    # all_joints = [
    #               "rh_WRJ1", "rh_WRJ2",
    #               "rh_EBJ",
    #               "rh_SHJ1", "rh_SHJ2"
    #               ]
    tol = 1e-1
    
    if not self.grabb_started:
      self.site_quat = self.desired_site_quat(physics, site_name, unit_direction, axis='y')
      self.grabb_started = True

    th_res = inv_kin.qpos_from_site_pose(physics, site_name=site_name, target_pos=target_point, target_quat = self.site_quat, joint_names=all_joints, inplace=False, rot_weight=0.5, tol=tol, max_steps=1000)
    
    if not th_res.success:
      print("Error! Could not find joint positions for th touch point", th_res)
      return
    
    for joint_name in all_joints:
      joint_idx = physics.model.name2id(joint_name, "joint")
      physics.data.qpos[joint_idx] = th_res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0

  def rotate_to_target(self, physics, object_name, geom_name, site_name="palm_skin_1"):
    mean_contact_point = self.calculate_mean_contact_point(physics, object_name, geom_name) # it should called only once
    target_point, unit_direction = self.calculate_normal_through_com(physics, mean_contact_point, geom_name, scale=3)
    print(target_point)
    end = target_point#physics.named.data.xpos[object_name] # should be surface
    th_start = physics.named.data.xpos["rh_thd_skin_1"]

    # all_joints = ["rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
    #               "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
    #               "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
    #               "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4",
    #               "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5",
    #               "rh_WRJ1", "rh_WRJ2",
    #               "rh_EBJ",
    #               "rh_SHJ1", "rh_SHJ2"
    #               ]
    
    all_joints = [
                  # "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
                  # "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
                  # "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
                  "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4",
                  "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5",
                  "rh_WRJ1", "rh_WRJ2",
                  "rh_EBJ",
                  "rh_EBJR",
                  # "rh_SHJ1", "rh_SHJ2"
                  ]
    tol = 1e-1
    
    if not self.grabb_started:
      self.site_quat = self.desired_site_quat(physics, site_name, unit_direction, axis='y')
      self.grabb_started = True

    th_res = inv_kin.qpos_from_site_pose(physics, site_name=site_name, target_quat = self.site_quat, joint_names=all_joints, inplace=False, tol=tol)
    
    if not th_res.success:
      print("Error! Could not find joint positions for th touch point", th_res)
      return
    
    ik_joint_indices = []
    for joint_name in all_joints:
      joint_idx = physics.model.name2id(joint_name, "joint")
      ik_joint_indices.append(joint_idx)
      physics.data.qpos[joint_idx] = th_res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0

    # Step 3: Lock all other joints by reapplying their original positions
    for idx in range(len(physics.data.qpos)):
      if idx not in ik_joint_indices:
        physics.data.qpos[idx] = self.current_qpos[idx]  # Restore original positions for non-IK joints


  def calculate_mean_contact_point(self, physics, object_name, geom_name):
    # In order to grab an object, we cannot grab at the contact surface, but the free surface
    # to calculate the thumb orientation, we need to calculate normal.
    # This will be the base of the normal.
    object_geom_id = physics.model.name2id(geom_name, "geom")
    # Collect contact points
    contact_points = []

    for contact in physics.data.contact:
        # Get the two geoms involved in the contact
        geom1 = contact.geom1
        geom2 = contact.geom2

        # Check if the object is involved in the contact
        if object_geom_id in (geom1, geom2):
            # Extract the contact position in world coordinates
            contact_pos = contact.pos
            contact_points.append(contact_pos)

    # Compute the mean of the contact points
    if contact_points:
        contact_points = np.array(contact_points)  # Convert to numpy array
        mean_contact_point = np.mean(contact_points, axis=0)
        print("Mean contact point:", mean_contact_point)
        return mean_contact_point
    else:
        print("No contacts found for object:", object_name)
    

  def calculate_normal_through_com(self, physics, mean_contact_point, geom_name, scale):
     # This function calculates a normal line that passes through the Cente rof Mass
     # The start of the line is the mean center of the contact points.
     # it is used to align the thumb and first finger for grabbing.
     # direction of thumb and ff should be parallel to the -direction 
     object_geom_id = physics.model.name2id(geom_name, "geom")
     if object_geom_id == -1:
        raise ValueError(f"Sphere with name '{geom_name}' not found.")
    
     # Get the position of the geom in world coordinates
     center = physics.data.geom_xpos[object_geom_id]
     radius = physics.model.geom_size[object_geom_id][0]
     print("center ", center, mean_contact_point, radius)
     direction = center - mean_contact_point
     # Step 2: Normalize the direction vector
     unit_direction = direction / np.linalg.norm(direction)
     # Step 3: Scale the direction vector
     target_vector = unit_direction * (scale * radius)
     # Step 4: Calculate the target point
     target_point = mean_contact_point + target_vector
     return target_point, unit_direction


  def grab_with_th_ff(self, physics, object_name):
    end = physics.named.data.xpos[object_name] # target position
    th_start = physics.named.data.xpos["rh_thd_skin_1"]
    ff_start = physics.named.data.xpos["rh_ffd_skin_1"]

    th_joint_names = ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"]
    ff_joint_names = ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4"]

    all_joints = ["rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
                  "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
                  "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
                  "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4",
                  "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5",
                  "rh_WRJ1", "rh_WRJ2",
                  "rh_EBJ",
                  "rh_SHJ1", "rh_SHJ2"
                  ]
    
    all_joints_idx = []
    for joint_name in all_joints:
      joint_idx = physics.model.name2id(joint_name, "joint")
      all_joints_idx.append(joint_idx)

    tol = 1e-4

    # this should be calculated only once
    if self.grab_with_th_ff_ray_th is None:
      self.grab_with_th_ff_ray_th = self.raycast(physics, start = th_start, end=end)
      if self.grab_with_th_ff_ray_th is None:
        print("Error! Could not find touch point for thumb")
        return

    print(self.grab_with_th_ff_ray_th)
    th_res = inv_kin.qpos_from_site_pose(physics, site_name="thd_skin_1", target_pos=self.grab_with_th_ff_ray_th, joint_names=th_joint_names, inplace=False, tol=tol)
    if not th_res.success:
      print("Error! Could not find joint positions for th touch point")
      return
    
    ik_joint_indices = []
    for joint_name in th_joint_names:
      joint_idx = physics.model.name2id(joint_name, "joint")
      physics.data.qpos[joint_idx] = th_res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0
      ik_joint_indices.append(joint_idx)

    # this should be calculated only once
    if self.grab_with_th_ff_ray_ff is None:
      self.grab_with_th_ff_ray_ff = self.raycast(physics, start = ff_start, end=end)
      if self.grab_with_th_ff_ray_ff is None:
        print("Error! Could not find touch point for first finger")
        return
      
    ff_res = inv_kin.qpos_from_site_pose(physics, site_name="ffd_skin_1", target_pos=self.grab_with_th_ff_ray_ff, joint_names=ff_joint_names, inplace=False, tol=tol)
    if not ff_res.success:
      print("Error! Could not find joint positions for ff touch point")
      return

    for joint_name in ff_joint_names:
      joint_idx = physics.model.name2id(joint_name, "joint")
      physics.data.qpos[joint_idx] = ff_res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0
      ik_joint_indices.append(joint_idx)

    # Lock all other joints by reapplying their original positions
    for idx in range(len(physics.data.qpos)):
      if idx not in ik_joint_indices and idx in all_joints_idx:
        physics.data.qpos[idx] = self.current_qpos[idx]  # Restore original positions for non-IK joints


  def ik_move_th_to_target(self, physics, start_th, target_name = "test_sphere"):
    end = physics.named.data.xpos[target_name] # target position
    start = physics.named.data.xpos["rh_thd_skin_1"] # effector position
    #body_id_to_exclude = physics.model.name2id('rh_thd_skin_1', "body")
    #hit_point = self.raycast(physics, start = start_th, end=end, body_id_to_exclude=body_id_to_exclude)
    # if hit_point is None:
    #    print("")
    #    return
    #hit_point = np.array(hit_point, dtype=np.float64).reshape(3)
    # joint_names = ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"]
    # res = inv_kin.qpos_from_site_pose(physics, site_name="thd_skin_1", target_pos=hit_point, joint_names=joint_names, inplace=False)
    # if not th_res.success:
    #   print("Could not find joint positions for touch point")
    #   return
    
    # joint_pos = []
    # ik_joint_indices = []
    # for joint_name in joint_names:
    #   joint_idx = physics.model.name2id(joint_name, "joint")
    #   physics.data.qpos[joint_idx] = res.qpos[joint_idx]
    #   physics.data.qvel[joint_idx] = 0
    #   ik_joint_indices.append(joint_idx)
    #   joint_pos.append(res.qpos[joint_idx])

    # # Step 3: Lock all other joints by reapplying their original positions
    # for idx in range(len(physics.data.qpos)):
    #   if idx not in ik_joint_indices:
    #     physics.data.qpos[idx] = self.current_qpos[idx]  # Restore original positions for non-IK joints

    # return joint_names, joint_pos

  def ik_move_ff_to_target(self, physics, start_ff, target_name = "test_sphere"):
    body_pos = physics.named.data.xpos["rh_ffd_skin_1"]
    body_id_to_exclude = physics.model.name2id('rh_ffd_skin_1', "body")
    print("body_id_to_exclude", body_id_to_exclude)
    target_position_world = physics.named.data.xpos[target_name] # target position
    target_body_id = physics.model.name2id(target_name, "body")
    print("target body id ", target_body_id)
    print("finger ", body_pos, "sphere ", target_position_world)
    hit_point = self.raycast(physics, start = start_ff, end=target_position_world, body_id_to_exclude=body_id_to_exclude)
    if hit_point is None:
       print("")
       return
    hit_point = np.array(hit_point, dtype=np.float64).reshape(3)
    print(hit_point, target_position_world)
    joint_names = [ "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4"]
    res = inv_kin.qpos_from_site_pose(physics, site_name="ffd_skin_1", target_pos=hit_point, joint_names=joint_names, inplace=False)
    print(res)
    if not res.success:
       return
    #self.grabbed = True
    joint_pos = []
    ik_joint_indices = []
    for joint_name in joint_names:
      joint_idx = physics.model.name2id(joint_name, "joint")
      physics.data.qpos[joint_idx] = res.qpos[joint_idx]
      physics.data.qvel[joint_idx] = 0
      ik_joint_indices.append(joint_idx)
      joint_pos.append(res.qpos[joint_idx])

    # Step 3: Lock all other joints by reapplying their original positions
    for idx in range(len(physics.data.qpos)):
      if idx not in ik_joint_indices:
        physics.data.qpos[idx] = self.current_qpos[idx]  # Restore original positions for non-IK joints

    return joint_names, joint_pos

  def test_ray(self, physics, start_name="object_1", end_name="object_2"):
     start = physics.named.data.xpos[start_name]
     end = physics.named.data.xpos[end_name]
     hit_point = self.raycast(physics, start, end, body_id_to_exclude=-1)


  def resting_hand(self, physics):
    flexed_finger = self.hand.relaxed_fingers()
    self.set_actuator_controls(flexed_finger, physics)

  def initialize_episode(self, physics):
    """Sets the state of the environment at the start of each episode.

    Initializes the cart and pole according to `swing_up`, and in both cases
    adds a small random initial velocity to break symmetry.

    Args:
      physics: An instance of `Physics`.
    """  
    
    self.resting_hand(physics)
    self.current_qpos = physics.data.qpos.copy()

    #self.grab_with_th_ff_ray_th = None
    #self.grab_with_th_ff_ray_ff = None


    super().initialize_episode(physics)  

  def before_step(self, action, physics):

      time = physics.data.time
      

      self.grab_with_th_ff_ray_th = None
      self.grab_with_th_ff_ray_ff = None
      
      
      
      if time  >= 1.0 and time  < 1.5:
        self.current_qpos = physics.data.qpos.copy()
        self.move_site_to_target(physics, object_name = 'ball_1', geom_name='geom_ball_1', site_name="palm_skin_1")
      elif time  >= 1.5 and time  < 2.0:  
        self.grab_with_th_ff_new(physics, object_name = 'ball_1', geom_name='geom_ball_1',)
      #    #self.current_qpos = physics.data.qpos.copy()
      #    self.rotate_to_target(physics, object_name = 'ball_1', geom_name='geom_ball_1', site_name="palm_skin_1")

      

        #self.th_joint_names, self.th_joint_pos = self.ik_move_th_to_target(physics, start_th)
        #self.ff_joint_names, self.ff_joint_pos = self.ik_move_ff_to_target(physics, start_ff)

      # if time  >= 2:
      #   self.ff_joint_names, self.ff_joint_pos = self.ik_move_ff_to_target(physics, start_ff)

        #self.grabbed = True

      # else:
      #    print(self.ff_joint_names, self.th_joint_names)
      #    if self.ff_joint_names:
      #     for i, ff_joint_name in enumerate(self.ff_joint_names):
      #         joint_idx = physics.model.name2id(ff_joint_name, "joint")
      #         physics.data.qpos[joint_idx] = self.ff_joint_pos[i]
      #    if self.th_joint_names:
      #     for i, th_joint_name in enumerate(self.th_joint_names):
      #         joint_idx = physics.model.name2id(th_joint_name, "joint")
      #         physics.data.qpos[joint_idx] = self.th_joint_pos[i]

      # if self.grabbed:
      #    print(physics.data.qpos)
      return
      flexed_finger = self.hand.touch()
      action_copy = np.copy(action)
      self.set_actions(flexed_finger, action_copy, physics)
      physics.set_control(action_copy)

  def get_observation(self, physics):
    """Returns an observation of the (bounded) physics state."""
    obs = collections.OrderedDict()
    obs['position'] = physics.named.data.qpos
    obs['velocity'] = physics.named.data.qvel
    return obs
  
  def get_reward(self, physics):
    """Returns a sparse or a smooth reward, as specified in the constructor."""
    return 1#
  
