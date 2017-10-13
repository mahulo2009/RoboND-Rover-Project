import numpy as np
from supporting_functions import normalize_angle,filter_steer_correction
    

class StrategyRock:
    def __init__(self, Rover):
        self.rover = Rover
        
    def select_steer(self):
        rock_angles_mean = np.mean(self.rover.rock_angles) 
        steer = np.clip(rock_angles_mean* 180/np.pi, -15, 15)
        steer_filter = filter_steer_correction(steer,stare_angle_threshold = 3)
        #print ('Select rock_angles_mean= ',rock_angles_mean)
        return steer_filter
        
    def is_detected(self):
        # Threshold to decide if we have found a rock
        pickup_rock_threshold = 0
        # TODO
        pickup_rock_angle_threshold = 3
        #print ('rover_rock_detect len(Rover.rock_angles)= ',len(Rover.rock_angles))
        if len(self.rover.rock_angles) > pickup_rock_threshold:
            #If the rock is on the Left, do not go for it.
            steer = self.select_steer()
            if steer > pickup_rock_angle_threshold:
                return False
            else: 
                return True
        else: 
            return False

    def is_reacheable(self):
        if (len (self.rover.rock_angles)!=0):
            rock_angles_mean = np.mean(self.rover.rock_angles)
            navigable_angle_index_ = self.rover.nav_angles < rock_angles_mean
            nav_angles = self.rover.nav_angles[navigable_angle_index_]
            #print ('Reachable rock_angles_mean= ',rock_angles_mean,' nav_angles= ',len(nav_angles))
            if len(nav_angles) > 0:
                return True
            else: 
                return False
        else:
            return False

    def is_close(self):
        #print ('rover_close_to_rock_check np.min(Rover.rock_dists) ',np.min(Rover.rock_dists))
        if np.min(self.rover.rock_dists) < 10:
            return True
        else:
            return False

class StrategyHome:
    def __init__(self, Rover):
        self.rover = Rover
        self.home_pos_x = 99.7 
        self.home_pos_y = 85.6
        
    def select_steer(self):    
        angle_world_home = normalize_angle(np.arctan2(self.home_pos_y - self.rover.pos[1], self.home_pos_x - self.rover.pos[0]) * 180/np.pi) 
        angle_rover_home = angle_world_home - self.rover.yaw  
        angle_clipped = np.clip(angle_rover_home, -15, 15)
        angle_filter = filter_steer_correction(angle_clipped)
        #print ('angle_world_home = ' , angle_world_home, 'angle_rover_home = ' , angle_rover_home ,' angle_clipped = ' , angle_clipped , 'angle_filter = ', angle_filter)
        return angle_filter    
        
    def is_detected(self):
        distance = np.sqrt((self.home_pos_x - self.rover.pos[0])**2 + (self.home_pos_y - self.rover.pos[1])**2)
        #print ('rover_home_detected = ' , distance)
        if distance < 5:
            return True
        else:
            return False

    def is_reacheable(self):
        return True

    def is_close(self):
        distance = np.sqrt((self.home_pos_x - self.rover.pos[0])**2 + \
                                        (self.home_pos_y - self.rover.pos[1])**2)
        #print ('rover_close_to_home_check = ' , distance)
        if distance < 1:
            return True
        else:
            return False
        