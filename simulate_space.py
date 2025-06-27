from dm_control import mjcf

# class to simulate the space seen by the camera
class SimulateSpace:
    def __init__(self):
        xml_path='./shadow_hand/right_arm_test.xml'
        self.model = self.mjcf_model_from_path(xml_path)
        
    def mjcf_model_from_path(self, xml_path): 
        mjcf_model = mjcf.from_path(xml_path)
        return mjcf_model

    def mjcf_model_from_string(self):
        XML=r"""
            <mujoco>
                <worldbody>
                </worldbody>
            </mujoco>
        """
        model = mjcf.from_xml_string(XML)
        return model
    
    def add_sphere_to_model(self, radius=0.05, pos=(0, 0, 1), rgba=(1, 0, 0, 1), name='test_sphere', geom_name="", free=False):
        new_body = self.model.worldbody.add(
            'body', name=name, pos=pos
        )

        if free:
            new_body.add("freejoint")

        new_body.add(
            'geom', name = geom_name, type='sphere', size=[radius], rgba=[1, 0, 0, 1]
        )
        
    def add_box_to_model(self, size = (0.05, 0.05, 0.05), pos=(0, 0, 1), rgba=(1, 1, 1, 1), name='test_box', geom_name=""):
        new_body = self.model.worldbody.add(
            'body', name=name, pos=pos
        )

        new_body.add(
            'geom', 
            name = geom_name, 
            type='box', 
            size=[0.1, 0.1, 0.1],  # Define the half-extents along each axis (x, y, z)
            rgba=rgba
        )