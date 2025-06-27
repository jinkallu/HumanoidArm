# import numpy as np

# # Graphics-related
# import matplotlib
# import matplotlib.animation as animation
# import matplotlib.pyplot as plt

# import PIL.Image

# import pyvista as pv

# from models import PCN

from dm_simulation import DMSimulation
from my_detectron2 import MyDetectron2
from pc_predictor import PCPredictor

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from dm_control.viewer import launch
from my_custom_env import MyCustomEnv

from dm_control import mujoco

from rest_task import RestTask
from dm_control.rl import control

from dm_control import mjcf

#from dm_control.viewer import viewer



import matplotlib.pyplot as plt

import open3d as o3d
import cv2

import math

from simulate_space import SimulateSpace


def display_mask(img_copy, pixels):
    # Colorize the mask in red (can be any color)
    img_copy[pixels] = [255, 0, 0]  # Set mask pixels to red

    # Display the image with the mask overlaid
    plt.imshow(img_copy)
    plt.axis('off')
    plt.show()

# isolate detected object
def object_depth_img(depth_img, pixels):
    #print(pixels)
    depth_object_only = np.zeros_like(depth_img)
    #depth_object_only[pixels] = depth_img[pixels]
    #valid_pixels = [(y, x) for y, x in pixels if isinstance(y, int) and isinstance(x, int)]
    valid_pixels = []
    for p in pixels:
        if isinstance(p, tuple) and len(p) == 2 and all(isinstance(i, int) for i in p):
            valid_pixels.append(p)
        else:
            print(f"Invalid entry found and ignored: {p}")
    rows, cols = pixels  # Separates the list of (y, x) tuples into rows and cols
    depth_object_only[rows, cols] = depth_img[rows, cols]
    return depth_object_only


def display_depth_img(img):
    plt.imshow(img, cmap='gray')
    plt.axis('off')
    plt.show()

def calculate_focal_lengths(fovy, image_width, image_height):
    # Convert FOV from degrees to radians
    fovy_rad = np.radians(fovy)
    
    # Calculate fy
    fy = image_height / (2 * np.tan(fovy_rad / 2))
    
    # Calculate fx based on the aspect ratio
    aspect_ratio = image_width / image_height
    fx = fy * aspect_ratio
    
    return fx, fy

def img2DTo3DPoint(depth_image, camera, u, v, r):
    image_height, image_width = depth_image.shape[:2]
    fov = camera.fovy[0]
    fx, fy = calculate_focal_lengths(fov, image_width, image_height)
    cx = image_width / 2  # Principal point x-coordinate
    cy = image_height / 2  # Principal point y-coordinate

    z = depth_image[v, u]  # Depth value at (u, v)
    # Calculate the 3D coordinates
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    u1 = u + r
    z1 = depth_image[v, u1]  # Depth value at (u, v)
    # Calculate the 3D coordinates
    x1 = (u1 - cx) * z1 / fx
    y1 = (v - cy) * z1 / fy

    r1 = math.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)


    return x, y, z, r1

def convert_normalized_depth_to_real_depth(z_normalized, near, far):
    # Convert normalized depth to real-world depth
    z_real = (z_normalized * (far - near) + near * far) / (far - z_normalized * (far - near))
    return z_real


def depth_img_to_pointcloud(depth_image, camera, model):
    image_height, image_width = depth_image.shape[:2]
    fov = camera.fovy[0]
    #near, far = model.vis.map.znear, model.vis.map.zfar  # Get near/far clipping planes from the camera
    #print("near, far ", near, far)
    fx, fy = calculate_focal_lengths(fov, image_width, image_height)
    cx = image_width / 2  # Principal point x-coordinate
    cy = image_height / 2  # Principal point y-coordinate

    # Create an array to hold the point cloud
    points = []

    # Iterate through each non-zero pixel in the depth image
    non_zero_pixels = np.where(depth_image > 0)  # Get the non-zero pixel coordinates
    
    for v, u in zip(*non_zero_pixels):  # For each non-zero pixel
        z = depth_image[v, u]
        # z = convert_normalized_depth_to_real_depth(z_normalized, near, far)
        # if z > 1:
        #     continue

        # Calculate the 3D coordinates
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Append the point (x, y, z) to the point cloud
        points.append([x, y, z])

    # Convert to a NumPy array
    point_cloud = np.array(points)

    return point_cloud

def depth_img_to_pointcloud_new(depth_image, camera, physics):
    image_height, image_width = depth_image.shape[:2]

    fov = physics.model.cam_fovy[camera.id]
    pos = physics.data.cam_xpos[camera.id]    # Camera position in world coordinates
    rot = physics.data.cam_xmat[camera.id].reshape(3, 3)  # Camera rotation matrix (transposed)

    # Example: flipping Y and Z, because z up in mujoco and y up in other systems
    adjustment_matrix = np.diag([1, -1, -1])
    rotation_matrix_adjusted = rot @ adjustment_matrix

    #print("cam ", pos)
    #print( rotation_matrix_adjusted)
    #pos_t = pos.T
    #print("pos inv ", pos_t)
    #rot_inv = np.linalg.inv(rot)
    #print(rot_inv)

    # Focal lengths
    fx, fy = calculate_focal_lengths(fov, image_width, image_height)
    cx = image_width / 2  # Principal point x-coordinate
    cy = image_height / 2  # Principal point y-coordinate

    

    # Create an array to hold the point cloud
    points = []

    # Iterate through each non-zero pixel in the depth image
    non_zero_pixels = np.where(depth_image > 0)

#     cam_mat = o3d.camera.PinholeCameraIntrinsic(image_width, image_height, fx, fy, cx, cy)
#     o3d_depth = o3d.geometry.Image(np.ascontiguousarray(depth_image))
#     position = camera.pos
#     quaternion = camera.quat
#     # Convert quaternion to a rotation matrix
#     rotation_matrix = R.from_quat(quaternion).as_matrix()

#     print("cam ", position)
#     print(rotation_matrix)


#     extrinsic_matrix = np.eye(4)
#     extrinsic_matrix[:3, :3] = rotation_matrix # Top-left 3x3 is the rotation matrix
#     extrinsic_matrix[:3, 3] = position # Top-right 3x1 is the translation vector

#     z_to_y_up = np.array([
#     [1, 0, 0, 0],
#     [0, 0, -1, 0],
#     [0, 1, 0, 0],
#     [0, 0, 0, 1]
# ])
    
#    # extrinsic_matrix = z_to_y_up @ extrinsic_matrix
#     #extrinsic_matrix = np.linalg.inv(extrinsic_matrix)

#     o3d_cloud = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, cam_mat, extrinsic=extrinsic_matrix)

#     o3d.visualization.draw_geometries([o3d_cloud])

#     print(cam_mat)


    for v, u in zip(*non_zero_pixels):  # For each non-zero pixel
        z = depth_image[v, u]
        
        # Calculate the 3D coordinates in the camera's frame
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        camera_coords = np.array([x, y, z])

        # Convert to world coordinates
        world_coords = rotation_matrix_adjusted @ camera_coords + pos
        points.append(world_coords)

    # Convert to a NumPy array
    point_cloud = np.array(points)
    return point_cloud

    #return np.asarray(o3d_cloud.points)



def count_inliers(params, points, threshold):
    a, b, c, r = params
    distances = np.sqrt((points[:, 0] - a)**2 + (points[:, 1] - b)**2 + (points[:, 2] - c)**2) - r
    inliers = np.sum(np.abs(distances) < threshold)  # Count inliers within the threshold
    return inliers

def sphere_loss(params, points):
    a, b, c, r = params
    return np.sum((np.sqrt((points[:, 0] - a)**2 + (points[:, 1] - b)**2 + (points[:, 2] - c)**2) - r)**2)

def ellipsoid_loss(params, points):
    a, b, c, r_x, r_y, r_z = params  # Ellipsoid parameters
    # Calculate the squared differences
    distances = ((points[:, 0] - a)**2 / r_x**2) + ((points[:, 1] - b)**2 / r_y**2) + ((points[:, 2] - c)**2 / r_z**2)
    #return np.sum((distances - 1)**2)  # Return the sum of squared differences
    return np.sum((distances - 1)**2) / len(points)

def generate_sphere_point_cloud(center, radius, num_points=1000):
    # Generate random spherical coordinates
    phi = np.random.uniform(0, 2 * np.pi, num_points)  # Azimuthal angle
    theta = np.random.uniform(0, np.pi, num_points)     # Polar angle

    # Convert spherical coordinates to Cartesian coordinates
    x = center[0] + radius * np.sin(theta) * np.cos(phi)
    y = center[1] + radius * np.sin(theta) * np.sin(phi)
    z = center[2] + radius * np.cos(theta)

    # Combine into a point cloud
    point_cloud = np.column_stack((x, y, z))
    
    return point_cloud

def generate_ellipsoid_point_cloud(center, radii, num_points=1000):
    """
    Generate a point cloud representing an ellipsoid.

    :param center: The center of the ellipsoid (x, y, z).
    :param radii: The semi-axis lengths of the ellipsoid (r_x, r_y, r_z).
    :param num_points: The number of points to generate.
    :return: A point cloud as a NumPy array of shape (num_points, 3).
    """
    # Generate random spherical coordinates
    phi = np.random.uniform(0, 2 * np.pi, num_points)  # Azimuthal angle
    cos_theta = np.random.uniform(-1, 1, num_points)   # Cosine of polar angle
    sin_theta = np.sqrt(1 - cos_theta**2)              # Sine of polar angle

    # Convert spherical coordinates to Cartesian coordinates, scaled by radii
    x = center[0] + radii[0] * sin_theta * np.cos(phi)
    y = center[1] + radii[1] * sin_theta * np.sin(phi)
    z = center[2] + radii[2] * cos_theta

    # Combine into a point cloud
    point_cloud = np.column_stack((x, y, z))
    
    return point_cloud

def fit2DEllipse(img):
    if len(img.shape) == 3:  # Check if the image has 3 channels (RGB)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Normalize the float32 image to range [0, 255]
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)

    # Convert the normalized image to uint8 (8-bit)
    img = img.astype(np.uint8)

    #img = np.array(pixels, dtype=np.uint8)
    _, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Fit an ellipse to the largest contour (assuming it's the ellipse)
    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)  # Largest contour
        ellipse = cv2.fitEllipse(contour)  # Fit ellipse
        # Extract ellipse parameters and print them
        center, axes, angle = ellipse
        print(f"Center: {center}")
        print(f"Axes: {axes}")
        print(f"Angle: {angle}")

        # Draw the fitted ellipse on the original image
        result_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.ellipse(result_image, ellipse, (0, 255, 0), 2)

        # Display the image
        plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
        plt.title('Fitted Ellipse')
        plt.show()

        return center, [axes[0] / 2, axes[1] / 2]



def stepp(time_step):
    print("Hello")
    physics.step()  

def add_sphere_to_model(mjcf_model, radius=0.05, pos=(0, 0, 1), rgba=(1, 0, 0, 1), name='test_sphere', geom_name="", free=False):
    new_body = mjcf_model.worldbody.add(
        'body', name=name, pos=pos
    )

    if free:
        new_body.add("freejoint")

    new_body.add(
        'geom', name = geom_name, type='sphere', size=[radius], rgba=[1, 0, 0, 1]
    )

def add_box_to_model(mjcf_model, size = (0.05, 0.05, 0.05), pos=(0, 0, 1), rgba=(1, 1, 1, 1), name='test_box', geom_name=""):
    new_body = mjcf_model.worldbody.add(
        'body', name=name, pos=pos
    )

    new_body.add(
        'geom', 
        name = geom_name, 
        type='box', 
        size=[0.1, 0.1, 0.1],  # Define the half-extents along each axis (x, y, z)
        rgba=rgba
    )


def add_arraow(mjcf_model):
    arrow_start = np.array([0.02515876, -0.03139905,  0.00275454])
    direction  = np.array([0.48243599, 0.18824651, 0.85546406])
    scale = 0.1
    arrow_end = arrow_start + direction * scale
    arrow_body = mjcf_model.worldbody.add('body', name='ray_arrow')
    arrow_geom = arrow_body.add(
        'geom', type='capsule', 
        fromto=[arrow_start[0], arrow_start[1], arrow_start[2], 
                arrow_end[0], arrow_end[1], arrow_end[2]], 
        size=[0.002],  # Arrow radius
        rgba=[0, 1, 0, 1]
    )

    
def body_world_to_local_coordinates(point, name='rh_palm'):
    body_orientation_matrix = physics.named.data.xmat[name].reshape(3, 3)
    # Inverse of the orientation matrix (3x3) for the rotation
    inv_orientation_matrix = np.linalg.inv(body_orientation_matrix)
    # Then, apply the inverse rotation matrix to get local coordinates
    point_local = np.dot(inv_orientation_matrix, point)

    return point_local

def body_local_to_world_coordinates(point_local, name='rh_palm'):
    #body_position_world = physics.named.data.xpos[name]
    body_orientation_matrix = physics.named.data.xmat[name].reshape(3, 3)
    body_position_world = np.dot(body_orientation_matrix, point_local)
    return body_position_world

def display_rgb(rgb):
        plt.imshow(rgb)
        plt.axis('off')  # Turn off axis labels
        plt.show()

def display_depthImg(depth_img):
        plt.imshow(depth_img)
        plt.axis('off')  # Turn off axis labels
        plt.show()

def mjcf_model_from_string():
    XML=r"""
        <mujoco>
        <worldbody>
            <body name="object_1" pos="0.29 -0.13 0.31">
                <geom name="object_1_geom" type="sphere" size="0.03" rgba="0.5 0.7 0.5 1" condim="6" priority="1" friction="0.5 0.01 0.003"/>
            </body>

            <body name="object_2" pos="0.29 -0.13 0.81">
                <geom name="object_2_geom" type="sphere" size="0.03" rgba="0.5 0.7 0.5 1" condim="6" priority="1" friction="0.5 0.01 0.003"/>
            </body>
        </worldbody>
        </mujoco>
    """
    model = mjcf.from_xml_string(XML)
    return model


def test_ray():
    mjcf_model = mjcf_model_from_string()
    physics = mjcf.Physics.from_mjcf_model(mjcf_model)
    task = RestTask(rest=True)
    time_limit = 5
    env = control.Environment(physics, task, time_limit=time_limit)
    launch(environment_loader=lambda: env)

def center_of_mass(point_cloud):
    masses = np.ones(point_cloud.shape[0])  # Uniform mass for each point
    weighted_coords = point_cloud * masses[:, np.newaxis]  # Element-wise multiplication
    com = np.sum(weighted_coords, axis=0) / np.sum(masses)
    return com
#test_ray()

#exit()


# simulation setup
xml_path='./shadow_hand/right_arm_test.xml'
mjcf_model = mjcf.from_path(xml_path)
pos = (0.091, -0.0325, 0.1)#(0.01, 0., 0.19299)
radius = 0.05
#add_sphere_to_model(mjcf_model, radius, pos=pos, geom_name="geom_test_sphere")
size = (0.05, 0.05, 0.05)
box_pos = (0.2, -0.4, 0.1)
add_box_to_model(mjcf_model, size, pos=box_pos, geom_name="geom_test_box")
sphere_pos = (0.2, -0.4, 0.23)
add_sphere_to_model(mjcf_model, radius=0.03, pos=sphere_pos, name="ball_1", geom_name="geom_ball_1", free=True)
#add_arraow(mjcf_model)
#add_sphere_to_model(mjcf_model, radius=0.005, pos=(0.05639994, -0.03189756, 0.02372854), name="test_surface")
#physics = mujoco.Physics.from_xml_path(xml_path)
physics = mjcf.Physics.from_mjcf_model(mjcf_model)

# Example usage in a running MuJoCo simulation
task = RestTask(rest=True)


time_limit= 2

name='rh_palm'
body_position_world = physics.named.data.xpos[name]
print("Palm position world", body_position_world)

body_position_local = body_world_to_local_coordinates(body_position_world, name)
print("Palm position local", body_position_local)

point_bet_th_ff_pos_loc = body_position_local.copy()
point_offset_local = (0.03, -0.05, 0.06)
point_bet_th_ff_pos_loc = np.add(point_bet_th_ff_pos_loc, point_offset_local)

print("Point in Palm between TH and FF world", body_local_to_world_coordinates(point_bet_th_ff_pos_loc))

# calculate control values for thumb actuators to reach target
# we are using mujoc invers kinematics
# it returns only joint positions, but what we need is actuator controls
# but we can access actuator controls, if we use the calculation inplace, so that we can access teh values from physics
# we copy physics first
# test_sphere_position_world = physics.named.data.xpos["test_sphere"] # target position
# joint_names = ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"]
# actuator_names = ["rh_A_THJ1", "rh_A_THJ2", "rh_A_THJ3", "rh_A_THJ4", "rh_A_THJ5"]

# actuator_index = physics.model.name2id(actuator_names[0], "actuator")

# physics_tmp = physics.copy(share_model=True)
# res = inverse_kinematics.qpos_from_site_pose(physics_tmp, site_name="thd_skin_1", target_pos=test_sphere_position_world, joint_names=joint_names, inplace=True)

# print(res)
#env = MyCustomEnv(physics, task=task, time_limit=time_limit)
env = control.Environment(physics, task, time_limit=time_limit)
# Access the number of actuators
num_actuators = env.physics.model.nu  # nu gives the number of actuators

# action_spec = env.action_spec()
# print(action_spec)
# def random_policy(time_step):
#   del time_step  # Unused.
#   return np.random.uniform(low=action_spec.minimum,
#                            high=action_spec.maximum,
#                            size=action_spec.shape)
#env.reset()
#env.reset()
#for i in range(200):
#    env.physics.step()
# for i in range(100):
#     env.physics.step()



# launch(environment_loader=lambda: env, policy=random_policy(env))
#launch(env)
# for i in range(100):
#     env.physics.step()
# for i in range(100):
#     print(env._step(None))

#launch(environment_loader=lambda: env)
# tmp exit
#exit()
#print("Camera Position:", env.sim.data.cam_xpos[0]) 

camera = physics.model.camera("fixed_camera") # fixed camera defined in the model


print(camera)
rgb_img = physics.render(camera_id=camera.id) # get RGB image
depth_img = physics.render(depth=True, camera_id=camera.id) # get depth image

display_rgb(rgb_img)


detectron = MyDetectron2()
detected = detectron.predict(rgb_img, display=True) # detect object from RGB image and corresponding mask
print(detected.class_names)

masks = detected.pred_masks if detected.has("pred_masks") else None

img_copy = rgb_img.copy()

pc_pred = PCPredictor(pcn_checkpoint_path='./pcn_model/pcn_checkpoint/best_l1_cd.pth')

simulated_space = SimulateSpace()
pos_tmp = (0.091, -0.0325, 0.1)#(0.01, 0., 0.19299)
radius_tmp = 0.05
#simulated_space.add_sphere_to_model(radius=radius_tmp, pos=pos_tmp, geom_name="geom_test_sphere")

coms = []
for i in range(len(masks)):
    if detected.class_names[i] != 'sports ball': # check for detected sports ball
        continue
    mask = masks[i].cpu().numpy()  # Get the mask as a numpy array
    pixels = np.where(mask == 1) 
    img_copy[pixels] = [255, 0, 0]  # Set mask pixels to red
    display_mask(img_copy, pixels)

    object_depth = object_depth_img(depth_img, pixels)
    #center, radius =  fit2DEllipse(object_depth)
    #print(center, radius) # which axes are they? 

    display_depth_img(depth_img) 
    display_depth_img(object_depth) 

    depth_img_copy = depth_img.copy()
    #x, y, z, r = img2DTo3DPoint(depth_img_copy, camera, int(center[0]), int(center[1]), max(int(radius[0]), int(radius[1])))

    #print("point ", x, y, z, "radius ", r)

    object_pc = depth_img_to_pointcloud_new(object_depth, camera, physics)
    print(object_pc)
    # Step 1: Estimate the center of the sphere
    sphere_center = np.mean(object_pc, axis=0)
    # Step 2: Estimate the radius
    distances = np.linalg.norm(object_pc - sphere_center, axis=1)
    estimated_radius = np.mean(distances)
    print("center est ", sphere_center, "radius ", estimated_radius)

    # Step 3: Filter points
    tolerance = 0.8  # Adjust tolerance as needed
    filtered_object_pc = object_pc[np.abs(distances - estimated_radius) < tolerance * estimated_radius]


    pc_pred.display_pointcloud(filtered_object_pc)

    initial_guess = [sphere_center[0], sphere_center[1], sphere_center[2], estimated_radius] # axes may be wrong from the circle fit
    result = minimize(sphere_loss, initial_guess, args=(filtered_object_pc,))
    print("Result ", result)
    fitted_sphere_point_cloud = generate_sphere_point_cloud(result.x[:3], result.x[3], 500)
    pc_pred.display_pointclouds(fitted_sphere_point_cloud, filtered_object_pc)
    #object_predicted = pc_pred.predict(filtered_object_pc)
    #pc_pred.display_pointcloud(object_predicted)
    obj_name = f"{detected.class_names[i]}_{i}"

    com = center_of_mass(fitted_sphere_point_cloud)
    coms.append({obj_name: com})
    print("COM", com)

    # now create a predicted model of the space
    print("Center ** ", result.x[:3], "rad ", result.x[3])
    print(f"{detected.class_names[i]}_{i}")
    if i == 0:
        simulated_space.add_sphere_to_model(radius=result.x[3], pos = result.x[:3], name=obj_name, geom_name=obj_name+"_geom", free=True)

simulated_space.add_box_to_model(size, pos=box_pos, geom_name="geom_test_box")    
simulated_physics = mjcf.Physics.from_mjcf_model(simulated_space.model)
sim_rgb_img = simulated_physics.render()
display_rgb(sim_rgb_img)

sim_task = RestTask(rest=True, coms=coms)
sim_env = control.Environment(simulated_physics, sim_task, time_limit=time_limit)
launch(environment_loader=lambda: sim_env)



exit()

simulation = DMSimulation(xml_path="./shadow_hand/right_arm_test.xml")
physics = simulation.get_physics()
camera = physics.model.camera("fixed_camera")
rgb_img = physics.render(camera_id=camera.id)
simulation.display_rgb(rgb_img)
depth_img = physics.render(depth=True, camera_id=camera.id)

simulation.display_depthImg(depth_img)

# Use PCN model to predict full object cloud from partial point cloud
pc_pred = PCPredictor(pcn_checkpoint_path='./pcn_model/pcn_checkpoint/best_l1_cd.pth')

# Detectron, identify objects in 2D RGB image
detectron = MyDetectron2()
detected = detectron.predict(rgb_img, display=True)
print(detected.class_names)
masks = detected.pred_masks if detected.has("pred_masks") else None
img_copy = rgb_img.copy()
for i in range(len(masks)):
    mask = masks[i].cpu().numpy()  # Get the mask as a numpy array
    pixels = np.where(mask == 1) 
    img_copy[pixels] = [255, 0, 0]  # Set mask pixels to red
    display_mask(img_copy, pixels) 
    #fit2DEllipse(img_copy)
    object_depth = object_depth_img(depth_img, pixels)
    center, radius =  fit2DEllipse(object_depth)
    display_depth_img(object_depth)
    object_pc = depth_img_to_pointcloud(object_depth, camera)
    pc_pred.display_pointcloud(object_pc)
    # Initial guess for center (a, b, c) and radius (r)
    initial_guess = [object_pc[0][0], object_pc[0][1], object_pc[0][2], 1]
    print("Sphere init guess ", initial_guess)
    print(object_pc)
    # Fitting the sphere
    result = minimize(sphere_loss, initial_guess, args=(object_pc,))
    print (result)
    center = [center[0] / 1000, center[1] / 1000]
    radius = [radius[0] / 1000, radius[1] / 1000]
    print(center, radius)
    initial_guess_ellipse = [center[0], center[1], result.x[2], radius[0], radius[1], result.x[3] * 0.9]  # Initial guess for ellipsoid parameters
    print("Ellipse init guess ", initial_guess_ellipse)
    bounds = [
    (result.x[0] - 1.0, result.x[0] + 1.0),  # Bounds for x-coordinate of the center
    (result.x[1] - 1.0, result.x[1] + 1.0),  # Bounds for y-coordinate of the center
    (result.x[2] - 1.0, result.x[2] + 1.0),  # Bounds for z-coordinate of the center
    (0.5 * result.x[3], 1.5 * result.x[3]),  # Bounds for x-axis radius
    (0.5 * result.x[3], 1.5 * result.x[3]),  # Bounds for y-axis radius
    (0.5 * result.x[3], 1.5 * result.x[3])   # Bounds for z-axis radius
]
    result_ellipse = minimize(ellipsoid_loss, initial_guess_ellipse, bounds = bounds, args=(object_pc,))
    # Count inliers
    #threshold_distance = 0.1  # Define an appropriate threshold
    #inliers_count = count_inliers(result.x, object_pc, threshold_distance)
    #print(inliers_count)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(object_pc)

    # Fit a sphere
    geom_type = o3d.geometry.PointCloud.get_geometry_type(point_cloud)
    print(geom_type)
    #sphere_center, sphere_radius = o3d.geometry.PointCloud.estimate_sphere(point_cloud)

    

    print(result_ellipse)
    fitted_sphere_point_cloud = generate_sphere_point_cloud(result.x[:3], result.x[3], 100)
    fitted_ellipsoid_point_cloud = generate_ellipsoid_point_cloud(result_ellipse.x[:3], result_ellipse.x[3:], 100)
    #fitted_sphere_point_cloud_2 = generate_sphere_point_cloud(result.x[:3], result.x[3] + 5, 100)
    pc_pred.display_pointclouds(fitted_sphere_point_cloud, object_pc)
    pc_pred.display_pointclouds(fitted_ellipsoid_point_cloud, object_pc)
    object_predicted = pc_pred.predict(object_pc)
    pc_pred.display_pointcloud(object_predicted)

# Find corresponding object positions in depth img

# convert object to point cloud


# Calculate finger pose according to object shape, size and use case

# Calculate minimum distance to the object cloud using RRT / Ray tracing

# Move end effector to object using Inverse Kinematics calculation

# Once the hand reaches, grab the object




