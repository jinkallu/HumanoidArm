from finger import Finger

class Hand:
    def __init__(self):
        self.finger = Finger()
    
    def flex_tendons(self, flex_val):
        positions = []
        for finger in self.finger.Type:
            if finger != self.finger.Type.Thumb:
                positions.extend(self.finger.flex_tendon(finger_type=finger, flex_val = flex_val))

        return positions
    
    def flex_MCPs(self, flex_val):
        positions = []
        for finger in self.finger.Type:
            positions.extend(self.finger.flex_MCP(finger_type = finger, flex_val = flex_val))

        return positions
    
    def flex_tendons_mcps(self, flex_val):
        positions = []
        positions.extend(self.flex_tendons(flex_val))
        positions.extend(self.flex_MCPs(flex_val))
        return positions
    
    def reset_fingers(self):
        positions = []
        positions.extend(self.finger.reset_lf())
        positions.extend(self.finger.reset_rf())
        positions.extend(self.finger.reset_mf())
        positions.extend(self.finger.reset_ff())
        return positions
    
    def relaxed_fingers(self):
        positions = []
        
        positions.extend(self.finger.relaxed_lf(tendon=0.6, MCP=0.3, knuckle=0, bottom=0))
        positions.extend(self.finger.relaxed_rf(tendon=0.6, MCP=0.3, knuckle=0))
        positions.extend(self.finger.relaxed_mf(tendon=0.6, MCP=0.3, knuckle=0))
        positions.extend(self.finger.relaxed_ff(tendon=0.6, MCP=0.3, knuckle=0))
        positions.extend(self.finger.relaxed_th(PIP=0.1, MCP=0.3, knuckle=0.0, j4=1.0, j5=-0.5))

        return positions
    
    def touch(self):
        positions = []

        positions.extend(self.finger.touch_th_finger(self.finger.Type.FirstFinger))

        return positions

    def get_hand_relaxed_pose(self):
        positions = []
        #positions.extend(self.flex_MCPs(3.14))
        #positions.extend(self.finger.flex_thumb_PIP(3.14))
        # positions.extend(self.finger.flex_MCP(self.finger.Type.Thumb, 3.14))
        # positions.extend(self.finger.move_rh_A_THJ4(1.22))
        # positions.extend(self.finger.move_rh_A_THJ5(1.04))
        # positions.extend(self.finger.move_rh_A_THJ3(0.209))
        positions.extend(self.touch())
        return positions
        
