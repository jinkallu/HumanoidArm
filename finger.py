
from enum import Enum

class Finger:
    class Type(Enum):
        LittleFinger = 1
        RingFinger = 2
        MiddleFinger = 3
        FirstFinger = 4
        Thumb = 5

    def __init__(self):
        pass

    def move_rh_A_LFJ0(self, flex_val): # ctrlrange="0 3.1415"
        return [{"type": "tendon", "name": "rh_A_LFJ0", "pos": flex_val}]
    
    def move_rh_A_LFJ3(self, flex_val): # ctrlrange="-0.261799 1.5708"
        return [{"type": "tendon", "name": "rh_A_LFJ3", "pos": flex_val}]
    
    def move_rh_A_LFJ4(self, flex_val): # ctrlrange="-0.349066 0.349066"
        return [{"type": "tendon", "name": "rh_A_LFJ4", "pos": flex_val}]
    
    def move_rh_A_LFJ5(self, flex_val): # ctrlrange="0 0.785398"
        return [{"type": "tendon", "name": "rh_A_LFJ5", "pos": flex_val}]
    
    def move_rh_A_RFJ0(self, flex_val): # ctrlrange="0 3.1415"
        return [{"type": "tendon", "name": "rh_A_RFJ0", "pos": flex_val}]
    
    def move_rh_A_RFJ3(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_RFJ3", "pos": flex_val}]
    
    def move_rh_A_RFJ4(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_RFJ4", "pos": flex_val}]
    
    def move_rh_A_MFJ0(self, flex_val): # ctrlrange="0 3.1415"
        return [{"type": "tendon", "name": "rh_A_MFJ0", "pos": flex_val}]
    
    def move_rh_A_MFJ3(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_MFJ3", "pos": flex_val}]
    
    def move_rh_A_MFJ4(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_MFJ4", "pos": flex_val}]
    
    def move_rh_A_FFJ0(self, flex_val): # ctrlrange="0 3.1415"
        return [{"type": "tendon", "name": "rh_A_FFJ0", "pos": flex_val}]
    
    def move_rh_A_FFJ3(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_FFJ3", "pos": flex_val}]
    
    def move_rh_A_FFJ4(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_FFJ4", "pos": flex_val}]
    
    def move_rh_A_THJ1(self, flex_val): # ctrlrange="-0.261799 1.5708"
        return [{"type": "tendon", "name": "rh_A_THJ1", "pos": flex_val}]
    
    def move_rh_A_THJ2(self, flex_val): # ctrlrange="-0.698132 0.698132"
        return [{"type": "tendon", "name": "rh_A_THJ2", "pos": flex_val}]
    
    def move_rh_A_THJ3(self, flex_val): # ctrlrange="-0.20944 0.20944"
        return [{"type": "tendon", "name": "rh_A_THJ3", "pos": flex_val}]
    
    def move_rh_A_THJ4(self, flex_val): # ctrlrange="0 1.22173" Moving from first finger to little finger, 1.22 towards little finger
        return [{"type": "tendon", "name": "rh_A_THJ4", "pos": flex_val}]
    
    def move_rh_A_THJ5(self, flex_val): # ctrlrange="-1.0472 1.0472" moving closer / away from fingers, 1.04, toward finger
        return [{"type": "tendon", "name": "rh_A_THJ5", "pos": flex_val}]
    
    def move_rh_A_WRJ1(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_WRJ1", "pos": flex_val}]
    
    def move_rh_A_WRJ2(self, flex_val):
        return [{"type": "tendon", "name": "rh_A_WRJ2", "pos": flex_val}]

    def reset_lf(self):
        positions = []

        positions.extend(self.move_rh_A_LFJ0(0))
        positions.extend(self.move_rh_A_LFJ3(0))
        positions.extend(self.move_rh_A_LFJ4(0))
        positions.extend(self.move_rh_A_LFJ5(0))
        
        return positions
    
    def reset_rf(self):
        positions = []

        positions.extend(self.move_rh_A_RFJ0(0))
        positions.extend(self.move_rh_A_RFJ3(0))
        positions.extend(self.move_rh_A_RFJ4(0))
           
        return positions
    
    def reset_mf(self):
        positions = []

        positions.extend(self.move_rh_A_MFJ0(0))
        positions.extend(self.move_rh_A_MFJ3(0))
        positions.extend(self.move_rh_A_MFJ4(0))
           
        return positions
    
    def reset_ff(self):
        positions = []

        positions.extend(self.move_rh_A_FFJ0(0))
        positions.extend(self.move_rh_A_FFJ3(0))
        positions.extend(self.move_rh_A_FFJ4(0))
           
        return positions

    def flex_tendon(self, finger_type, flex_val):
        positions = []

        if finger_type == self.Type.LittleFinger:
            positions.extend(self.move_rh_A_LFJ0(flex_val))
        elif finger_type == self.Type.RingFinger:
            positions.extend(self.move_rh_A_RFJ0(flex_val))
        elif finger_type == self.Type.MiddleFinger:  
            positions.extend(self.move_rh_A_MFJ0(flex_val))
        elif finger_type == self.Type.FirstFinger:  
            positions.extend(self.move_rh_A_FFJ0(flex_val))

        return positions
    
    def flex_MCP(self, finger_type, flex_val):
        positions = []

        if finger_type == self.Type.LittleFinger:
            positions.extend(self.move_rh_A_LFJ3(flex_val))
        elif finger_type == self.Type.RingFinger:
            positions.extend(self.move_rh_A_RFJ3(flex_val))
        elif finger_type == self.Type.MiddleFinger:  
            positions.extend(self.move_rh_A_MFJ3(flex_val))
        elif finger_type == self.Type.FirstFinger:  
            positions.extend(self.move_rh_A_FFJ3(flex_val))
        elif finger_type == self.Type.Thumb:
            positions.extend(self.move_rh_A_THJ2(flex_val))

        return positions
    
    def flex_thumb_PIP(self, flex_val):
        positions = []
        positions.extend(self.move_rh_A_THJ1(flex_val))
        return positions
    
    def relaxed_lf(self, tendon = 0.5, MCP = 0, knuckle = 0, bottom = 0):
        positions = []

        positions.extend(self.move_rh_A_LFJ0(tendon))
        positions.extend(self.move_rh_A_LFJ3(MCP))
        positions.extend(self.move_rh_A_LFJ4(knuckle))
        positions.extend(self.move_rh_A_LFJ5(bottom))

        return positions
    
    def relaxed_rf(self, tendon = 0.5, MCP = 0, knuckle = 0):
        positions = []

        positions.extend(self.move_rh_A_RFJ0(tendon))
        positions.extend(self.move_rh_A_RFJ3(MCP))
        positions.extend(self.move_rh_A_RFJ4(knuckle))

        return positions
    
    def relaxed_mf(self, tendon = 0.5, MCP = 0, knuckle = 0):
        positions = []

        positions.extend(self.move_rh_A_MFJ0(tendon))
        positions.extend(self.move_rh_A_MFJ3(MCP))
        positions.extend(self.move_rh_A_MFJ4(knuckle))

        return positions
    
    def relaxed_ff(self, tendon = 0.5, MCP = 0, knuckle = 0):
        positions = []

        positions.extend(self.move_rh_A_FFJ0(tendon))
        positions.extend(self.move_rh_A_FFJ3(MCP))
        positions.extend(self.move_rh_A_FFJ4(knuckle))

        return positions
    
    def relaxed_th(self, PIP = 0.5, MCP = 0, knuckle = 0, j4=0, j5=0):
        positions = []

        positions.extend(self.move_rh_A_THJ1(PIP))
        positions.extend(self.move_rh_A_THJ2(MCP))
        positions.extend(self.move_rh_A_THJ3(knuckle))
        positions.extend(self.move_rh_A_THJ4(j4))
        positions.extend(self.move_rh_A_THJ5(j5))

        return positions
    
    def touch_th_finger(self, type):
        # Touch with thumb to other finger tips
        positions = []

        if type == self.Type.FirstFinger:
            positions.extend(self.relaxed_ff(tendon=1.5, MCP=0.9, knuckle=0))
            positions.extend(self.relaxed_th(PIP=0.1, MCP=0.3, knuckle=0.1, j4=1.0, j5=0.3))

        elif type == self.Type.MiddleFinger:
            positions.extend(self.relaxed_mf(tendon=1.6, MCP=1.0, knuckle=0))
            positions.extend(self.relaxed_th(PIP=0.1, MCP=0.3, knuckle=0.1, j4=1.22, j5=0.5))

        elif type == self.Type.RingFinger:
            positions.extend(self.relaxed_rf(tendon=1.6, MCP=0.8, knuckle=0))
            positions.extend(self.relaxed_th(PIP=0.1, MCP=0.3, knuckle=0.1, j4=1.22, j5=0.8))

        elif type == self.Type.LittleFinger:
            positions.extend(self.relaxed_lf(tendon=2.0, MCP=0.2, knuckle=0, bottom=0.6))
            positions.extend(self.relaxed_th(PIP=0.1, MCP=0.1, knuckle=0.1, j4=1.22, j5=0.8))

        return positions




        


