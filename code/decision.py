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
    stare_angle_threshold = 5
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
    stare_angle_offset = -11
    #Filter navigable terrain points farther than 6 meters
    nav_angles = filter_navigable_angles(Rover)
    nav_angles_mean = np.mean(nav_angles * 180/np.pi)
    nav_angles_offset = nav_angles_mean + stare_angle_offset 
    steer = np.clip(nav_angles_offset, -15, 15)
    #print ('nav_angles_mean =',nav_angles_mean,' nav_angles_offset = ', nav_angles_offset, ' steer = ',steer )
    
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
    return result
    
# Select the direction to twist 
def rover_turn_unstuk_yaw(Rover):
    Rover.throttle = 0
    Rover.brake = 0
    
    yaw = normalize_angle(Rover.yaw)
    unstuck_yaw = normalize_angle(Rover.unstuck_yaw)
    if (yaw < unstuck_yaw):
        distance_1 = abs (yaw-unstuck_yaw)
        distance_2 = yaw + 360 - unstuck_yaw
    else:
        distance_1 = 360 - yaw + unstuck_yaw
        distance_2 = abs (yaw-unstuck_yaw)
    
    if  distance_1 < distance_2:
        Rover.steer = 15
    else:    
        Rover.steer = -15
    #print ('distance_1=',distance_1,'distance_2=',distance_2)    
    #print ('Rover.yaw=',normalize_angle(Rover.yaw),'Rover.unstuck_yaw=',normalize_angle(Rover.unstuck_yaw),'steer = ',steer)
        
# Check if the angle to go forward has been reached        
def rover_go_forward_unstack_check(Rover):
    yaw = normalize_angle(Rover.yaw)
    unstuck_yaw = normalize_angle(Rover.unstuck_yaw)
    angle_diference = abs(unstuck_yaw - yaw)
    #print('unstuck_yaw =',unstuck_yaw,'yaw =',yaw)
    if angle_diference < 5 :
        return True
    else:
        return False

def rover_select_rock_steer(Rover):
    for idx in range(len(Rover.samples_pos[0])):
        test_rock_x = Rover.samples_pos[0][idx]
        test_rock_y = Rover.samples_pos[1][idx]
        rock_sample_dists = np.sqrt((test_rock_x - Rover.pos[0])**2 + \
                            (test_rock_y - Rover.pos[1])**2)
        
        if np.min(rock_sample_dists) < 3:
            #print ( 'rover_x=', Rover.pos[0] , 'rover_y=', Rover.pos[1] ,'rock_x=',test_rock_x,'rock_y=',test_rock_y)    
            #print ( 'rover_x=', test_rock_x - Rover.pos[0] , 'rover_y=', test_rock_y - Rover.pos[1] )
            angle_world = math.degrees(np.arctan2(test_rock_y - Rover.pos[1], test_rock_x - Rover.pos[0]))
            #print (angle_world)
            angle_rover =  Rover.yaw - angle_world
            #print (angle_rover) 
            steer = -np.clip(angle_rover , -15, 15)
            #print (steer)
            return filter_steer_correction(steer)

def rover_close_to_rock_check(Rover):
    for idx in range(len(Rover.samples_pos[0])):
        test_rock_x = Rover.samples_pos[0][idx]
        test_rock_y = Rover.samples_pos[1][idx]
        rock_sample_dists = np.sqrt((test_rock_x - Rover.pos[0])**2 + \
                            (test_rock_y - Rover.pos[1])**2)
        if np.min(rock_sample_dists) < 3:
            return True
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
                # Check if there is navigable terrain ahead       
                if navigable_terrain_check(Rover,threshold=Rover.stop_forward):
                    # Check if we found a rock
                    #if rover_close_to_rock_check(Rover) :
                    #    Rover.mode = 'rockapproaching'
                    #else:
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
                # Twist the rover in the  unstuck  direction
                rover_turn_unstuk_yaw(Rover)
                # Go forward in the unstuck direction
                if rover_go_forward_unstack_check(Rover):
                    # Select the next unstuck direction
                    Rover.unstuck_yaw=random.uniform(0,360)
                    Rover.unstuck_yaw_last=Rover.yaw
                    Rover.mode = 'forward'
                    # Reset the stuck timer
                    Rover.stuck_position_time = time.time()
        elif Rover.mode == 'rockdetected':
            if rover_is_position_stuck_check(Rover):
                Rover.mode = 'stuck'
            else:
                if Rover.vel > 0.2:
                    rover_stop(Rover)
        elif Rover.mode == 'rockapproaching':
            if rover_is_position_stuck_check(Rover):
                Rover.mode = 'stuck'
            else:
                if navigable_terrain_check(Rover,threshold=Rover.stop_forward):
                    if rover_close_to_rock_check(Rover) :
                        # Set the rock approaching velocity
                        rover_set_velocity(Rover,0.2)
                        # Set the angle into the rock direction
                        Rover.steer = rover_select_rock_steer(Rover)
                        # If we are close enough, pick up the rock
                        if Rover.near_sample:
                            Rover.mode = 'pickuprock'
                    else:
                        Rover.mode = 'forward'
                else:
                    Rover.mode = 'stop'
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
    
    #Rover.throttle = 0
    #Rover.brake = 0
    #Rover.steer = 0
    
    return Rover