import math 
import time
import numpy as np
import random

# Filter navigable terrain points farther than 4 meters    
def filter_navigable_angles(Rover):
    distance_thredshold = 40
    index_distance_faraway = Rover.nav_dists < distance_thredshold
    nav_angles = Rover.nav_angles[index_distance_faraway]
    return nav_angles

#TODO
# Filter angle corrections smaller than five degrees    
def filter_steer_correction(steer,stare_angle_threshold = 5):
    # Do not apply small corrections to avoid oscilations        
    if abs(steer) < stare_angle_threshold:
        return 0
    else:
        return steer        

# Normalize the angle between 0 and 360
def normalize_angle(angle):
    result = angle
    while(result <0):
        result+=360
    while(result >360):
        result-=360
    return result


class RoverController:
    
    def __init__(self, Rover):
        self.rover = Rover
        
    # Accelerate until speed is reached 
    def set_velocity(self,velocity):
        self.rover.brake = 0
        if self.rover.vel <= velocity:
            # Set throttle value to throttle setting
            self.rover.throttle = velocity
        else: # Else coast
            self.rover.throttle = 0
            
    # Check if the velocity has been reached
    def is_reach_velocity(self,velocity):
        if self.rover.vel >= velocity:
            return True
        else: 
            return False

    # Twist the rover
    def twist(self):
        self.rover.throttle = 0
        self.rover.brake = 0
        self.rover.steer = 15 
 
    # Release throttle, brake set
    def stop(self):
        self.rover.throttle = 0
        self.rover.brake = self.rover.brake_set
        self.rover.steer = 0
        
    def is_stuck(self):
        # Threshold to identify if the rover is stuck
        stuck_time_threshold = 5
        #print ('Rover.vel = ',Rover.vel)
        if math.fabs(self.rover.vel) < 0.1:
            #Check for how long I have not been moving. 
            stuck_time = time.time() - self.rover.stuck_position_time
            #print('stuck_time = ',stuck_time)
            if stuck_time > stuck_time_threshold:
                return True
            else:
                return False
        else:
            # Reset the timer
            self.rover.stuck_position_time = time.time()
    
    # Check if there is navigable terrain ahead
    def is_navigable_terrain(self,threshold):
        #Filter navigable terrain points farther than 4 meters
        nav_angles = filter_navigable_angles(self.rover)
        # The number of points ahead is bigger than threshold
        if len(nav_angles) >= threshold:
            return True
        else: 
            return False
        
    # Select the steer angle to navigate
    def select_navigation_steer(self):
        # Standard deviation threshold to considere if we may be moving in circles
        std_angles_threshold = 28
        # Offset to the angle to follow the wall. 
        stare_angle_offset = -11
        # A random factor to avoid moving in circles.
        stare_angle_random_factor = 1
        #Filter navigable terrain points farther than 4 meters
        nav_angles = filter_navigable_angles(self.rover)
        # Mean of the angles
        nav_angles_mean = np.mean(nav_angles * 180/np.pi)
        # Standard desviation of the angles
        nav_angles_std = np.std(nav_angles * 180/np.pi)
        # If the Standard deviatio is high generate a random factor
        if (nav_angles_std > std_angles_threshold):
            print("nav_angles_std =", nav_angles_std)
            stare_angle_random_factor = random.randrange(-1,1)
        # Calculate the steer angle    
        steer_angle = nav_angles_mean + stare_angle_offset * stare_angle_random_factor
        # Clip then angle between -15 and 15. The values the rover accepts. 
        steer_clipped = np.clip(steer_angle, -15, 15)
        #print ('nav_angles_mean =',nav_angles_mean,' nav_angles_offset = ', nav_angles_offset, ' steer = ',steer )
        #Return the filter steer angle
        return filter_steer_correction(steer_clipped)


    # Select the steer angle to navigate
    def select_unstuk_yaw(self):
        self.rover.throttle = 0
        self.rover.brake = 0
        
        yaw = normalize_angle(self.rover.yaw)
        unstuck_yaw = normalize_angle(self.rover.unstuck_yaw)
        if (yaw < unstuck_yaw):
            distance_1 = abs (yaw-unstuck_yaw)
            distance_2 = yaw + 360 - unstuck_yaw
        else:
            distance_1 = 360 - yaw + unstuck_yaw
            distance_2 = abs (yaw-unstuck_yaw)
        
        if  distance_1 < distance_2:
            self.rover.steer = 15
        else:    
            self.rover.steer = -15
        #print ('distance_1=',distance_1,'distance_2=',distance_2)    
        #print ('Rover.yaw=',normalize_angle(Rover.yaw),'Rover.unstuck_yaw=',normalize_angle(Rover.unstuck_yaw),'steer = ',steer)


    # Check if the angle to go forward has been reached        
    def is_unstuk_yaw_reached(self):
        yaw = normalize_angle(self.rover.yaw)
        unstuck_yaw = normalize_angle(self.rover.unstuck_yaw)
        angle_diference = abs(unstuck_yaw - yaw)
        #print('unstuck_yaw =',unstuck_yaw,'yaw =',yaw)
        if angle_diference < 5 :
            return True
        else:
            return False

