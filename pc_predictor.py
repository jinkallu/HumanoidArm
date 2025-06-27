import torch
from pcn_model import PCN
import numpy as np
import pyvista as pv

class PCPredictor:
    def __init__(self, pcn_checkpoint_path):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = PCN(16384, 1024, 4).to(self.device)
        self.model.load_state_dict(torch.load(pcn_checkpoint_path, map_location=self.device))
        self.model.eval()  # Set the model to evaluation mode (optional)

    def predict(self, point_cloud):
        pc_tensor = torch.from_numpy(point_cloud).unsqueeze(0).float().to(self.device)  # Shape (1, 1024, 3)
        with torch.no_grad():
            predicted = self.model(pc_tensor)
            if isinstance(predicted, tuple):
                predicted = predicted[0]  # Access the first element
            
            predicted = predicted.squeeze(0).cpu().numpy()
            return predicted

    def display_pointcloud(self, point_cloud):
        # Create a PyVista point cloud object
        point_cloud_pv = pv.PolyData(point_cloud)
        # Plot the point cloud
        plotter = pv.Plotter()
        plotter.add_mesh(point_cloud_pv, point_size=10, color="red")
        plotter.show()   

    def display_pointclouds(self, point_cloud_1, point_cloud_2):
        # Create a PyVista point cloud object
        point_cloud_pv_1 = pv.PolyData(point_cloud_1)
        point_cloud_pv_2 = pv.PolyData(point_cloud_2)
        # Plot the point cloud
        plotter = pv.Plotter()
        plotter.add_mesh(point_cloud_pv_1, point_size=10, color="red", opacity=0.5)
        plotter.add_mesh(point_cloud_pv_2, point_size=10, color="blue", opacity=0.5)
        plotter.show_bounds(grid='front', location='outer')
        plotter.show()        



    def sample_sphere(self, num_points=1024, radius=1.0):
        phi = np.random.uniform(0, 2 * np.pi, num_points)  # Azimuthal angle
        costheta = np.random.uniform(-1, 1, num_points)    # Cosine of polar angle
        theta = np.arccos(costheta)                        # Polar angle
        
        x = radius * np.sin(theta) * np.cos(phi)
        y = radius * np.sin(theta) * np.sin(phi)
        z = radius * np.cos(theta)
        
        return np.vstack((x, y, z)).T  # Return as (num_points, 3)
        

if __name__ == "__main__":
    pc_pred = PCPredictor(pcn_checkpoint_path='./pcn_model/pcn_checkpoint/best_l1_cd.pth')
    sphere_points = pc_pred.sample_sphere(num_points=100, radius=1.0)
    predicted = pc_pred.predict(sphere_points)
    pc_pred.display_pointcloud(predicted)



