import numpy as np
import time
import math
import random

# Filter navigable terrain points farther than 4 meters    
def filter_navigable_angles(Rover):
    distance_thredshold = 40
    index_distance_faraway = Rover.nav_dists < distance_thredshold
    nav_angles = Rover.nav_angles[index_distance_faraway]
    
    return nav_angles

# Filter angle corrections smaller than five degrees    
def filter_steer_correction(steer):
    stare_angle_threshold = 3
    # Do not apply small corrections to avoid oscilations        
    if abs(steer) < stare_angle_threshold:
        return 0
    else:
        return steer        

# Check if the rover has been stuck in position during a certain time      
def rover_is_position_stuck_check(Rover):
    # Threshold to identify if the rover is stuck
    stuck_time_threshold = 5
    #print ('Rover.vel = ',Rover.vel)
    if math.fabs(Rover.vel) < 0.1:
        #Check for how long I have not been moving. 
        stuck_time = time.time() - Rover.stuck_position_time
        #print('stuck_time = ',stuck_time)
        if stuck_time > stuck_time_threshold:
            return True
        else:
            return False
    else:
        # Reset the timer
        Rover.stuck_position_time = time.time()
        
# Check if there is navigable terrain ahead         
def navigable_terrain_check(Rover,threshold):
    #Filter navigable terrain points farther than 6 meters
    nav_angles = filter_navigable_angles(Rover)
    # The number of points ahead is bigger than threshold
    if len(nav_angles) >= threshold:
        return True
    else: 
        return False
    
# Select the steer angle to navigate
def rover_select_navigation_steer(Rover):
    # Offset to the angle to follow the wall. 
    stare_angle_offset = -12
    #Filter navigable terrain points farther than 6 meters
    nav_angles = filter_navigable_angles(Rover)
    steer = np.clip(np.mean(nav_angles * 180/np.pi)+stare_angle_offset, -15, 15)
    #Return the steer angle
    return filter_steer_correction(steer)

# Accelerate until speed is reached 
def rover_set_velocity(Rover,velocity):
    Rover.brake = 0
    if Rover.vel <= velocity:
        # Set throttle value to throttle setting
        Rover.throttle = velocity
    else: # Else coast
        Rover.throttle = 0
        
# Check if the velocity has been reached
def rover_reach_velocity(Rover,velocity):
    if Rover.vel >= velocity:
        return True
    else: 
        return False
    
# Twist the rover
def rover_twist(Rover):
    Rover.throttle = 0
    Rover.brake = 0
    Rover.steer = 15 
 
# Release throttle, brake set
def rover_stop(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0

# Check if there is rock ahead
def rock_check(Rover):
    # Threshold to detect rocks
    pickup_rock_threshold = 5
    if len(Rover.rock_angles) > pickup_rock_threshold:
        return True
    else: 
        return False
        
# Select the steer angle to pick up a rock
def rover_select_rock_steer(Rover):
    angle = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
    return filter_steer_correction(angle)
    

def rock_detect(Rover):
    if rock_check(Rover):
        #If the rock is on the Left, do not go for it.
        if rover_select_rock_steer(Rover) > 0:
            return False
        else: 
            return True
    else: 
        return False


# Normalize the angle between -180 and 180
def normalize_angle(angle):
    result = angle
    while(result >360):
        result-=360
    if result > 180:
        result-= 360
    return result
    
# Select the direction to twist 
def rover_turn_unstuk_yaw(Rover):
    Rover.throttle = 0
    Rover.brake = 0
    unstuck_yaw = normalize_angle(Rover.unstuck_yaw)
    unstuck_yaw_last = normalize_angle(Rover.unstuck_yaw_last)
    if (unstuck_yaw - unstuck_yaw_last) < 0:
        Rover.steer = -15
    else: 
        Rover.steer = 15
        
# Check if the angle to go forward has been reached        
def rover_go_forward_unstack_check(Rover):
    unstuck_yaw = normalize_angle(Rover.unstuck_yaw)
    yaw = normalize_angle(Rover.yaw)
    angle_diference = abs(unstuck_yaw - yaw)
    
    print('unstuck_yaw =',unstuck_yaw,'yaw =',yaw,'angle_diference =',angle_diference )
    
    if angle_diference < 5 :
        return True
    else:
        return False
    
# Decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.mode == 'forward':
            # Check if we are stuck
            if rover_is_position_stuck_check(Rover):
                Rover.mode = 'stuck'
            else:
                # Check if we found a rock
                if rock_detect(Rover) :
                    Rover.mode = 'rockdetected'
                # Check if there is navigable terrain ahead       
                elif navigable_terrain_check(Rover,threshold=Rover.stop_forward):
                    # Set the navigable velocity
                    rover_set_velocity(Rover,Rover.throttle_set)
                    # Set the navigable angle
                    Rover.steer = rover_select_navigation_steer(Rover)
                else:
                    # There is not navigable terrain ahead, enter stop mode
                    Rover.mode = 'stop'
        elif Rover.mode == 'stop':
            # Reduce velocity if we are still moving 
            if Rover.vel > 0.2:
                rover_stop(Rover)
            else:
                # Twist the rover until navigable terrain ahead again
                rover_twist(Rover)
                if navigable_terrain_check(Rover,threshold=Rover.go_forward) :
                    Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            # Reduce velocity if we are still moving
            if Rover.vel > 0.2:
                rover_stop(Rover)
            else:
                #print ('Rover.unstuck_yaw = ',Rover.unstuck_yaw)
                # Twist the rover in the  unstuck  direction
                rover_turn_unstuk_yaw(Rover)
                # Go forward in the unstuck direction
                if rover_go_forward_unstack_check(Rover): 
                    Rover.mode = 'forward'
                    # Select the next unstuck direction
                    Rover.unstuck_yaw=random.uniform(0,360)
                    Rover.unstuck_yaw_last=Rover.yaw
                    # Reset the stuck timer
                    Rover.stuck_position_time = time.time()
        elif Rover.mode == 'rockdetected':
            if rover_is_position_stuck_check(Rover):
                Rover.mode = 'stuck'
            else:
                if Rover.vel > 0.2:
                    rover_stop(Rover)
                else:
                    Rover.mode = 'rockapproaching'
        elif Rover.mode == 'rockapproaching':
            if rover_is_position_stuck_check(Rover):
                Rover.mode = 'stuck'
            else:
                # Set the rock approaching velocity
                rover_set_velocity(Rover,0.2)
                if rock_check(Rover):
                    # Set the angle into the rock direction
                    Rover.steer = rover_select_rock_steer(Rover)
                    # If we are close enough, pick up the rock
                    if Rover.near_sample:
                        Rover.mode = 'pickuprock'
                #else:
                    # If the rock is missed, go forward
                    # Define timeout pickup the rock TODO
                    
        elif Rover.mode == 'pickuprock':                       
            # Stop the rover to pick up the rock
            rover_stop(Rover)
            # Pick up the rock
            if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
                Rover.send_pickup = True
            else:
                # Move to detect if the rock has been picked up                
                Rover.mode = 'finishpickuprock'
        elif Rover.mode == 'finishpickuprock':
            if  not Rover.picking_up:
                #Reset stuck time and go forward
                Rover.stuck_position_time = time.time() 
                Rover.mode = 'forward'

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        
    print("Rover.mode = ",Rover.mode," Rover.throttle = ",Rover.throttle," Rover.brake = ", Rover.brake," Rover.vel = ",Rover.vel," Rover.steer = ",Rover.steer)   
    
    return Rover