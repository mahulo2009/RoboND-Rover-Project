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
def filter_steer_correction(steer,stare_angle_threshold = 5):
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
    #Filter navigable terrain points farther than 4 meters
    nav_angles = filter_navigable_angles(Rover)
    # The number of points ahead is bigger than threshold
    if len(nav_angles) >= threshold:
        return True
    else: 
        return False
    
# Select the steer angle to navigate
def rover_select_navigation_steer(Rover):
    # Standard deviation threshold to considere if we may be moving in circles
    std_angles_threshold = 28
    # Offset to the angle to follow the wall. 
    stare_angle_offset = -11
    # A random factor to avoid moving in circles.
    stare_angle_random_factor = 1
    #Filter navigable terrain points farther than 4 meters
    nav_angles = filter_navigable_angles(Rover)
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
    
#TODO 
def rover_select_rock_steer(Rover):
    rock_angles_mean = np.mean(Rover.rock_angles) * 180/np.pi 
    steer = np.clip(rock_angles_mean* 180/np.pi, -15, 15)
    steer_filter = filter_steer_correction(steer,stare_angle_threshold = 3)
    #print ('Select rock_angles_mean= ',rock_angles_mean)
    return steer_filter

def rover_rock_reacheable_check(Rover): 
    if (len (Rover.rock_angles)!=0):
        rock_angles_mean = np.mean(Rover.rock_angles)
        navigable_angle_index_ = Rover.nav_angles < rock_angles_mean
        nav_angles = Rover.nav_angles[navigable_angle_index_]
        #print ('Reachable rock_angles_mean= ',rock_angles_mean,' nav_angles= ',len(nav_angles))
        if len(nav_angles) > 0:
            return True
        else: 
            return False
    else:
        return False

# Detect if we are close to a rock. The rover ignores the rock if it is on the left. 
def rover_rock_detected(Rover):
    # Threshold to decide if we have found a rock
    pickup_rock_threshold = 0
    pickup_rock_angle_threshold = 3
    #print ('rover_rock_detect len(Rover.rock_angles)= ',len(Rover.rock_angles))
    if len(Rover.rock_angles) > pickup_rock_threshold:
        #If the rock is on the Left, do not go for it.
        steer = rover_select_rock_steer(Rover)
        if steer > pickup_rock_angle_threshold:
            return False
        else: 
            return True
    else: 
        return False

# TODO
def rover_close_to_rock_check(Rover):
    #print ('rover_close_to_rock_check np.min(Rover.rock_dists) ',np.min(Rover.rock_dists))
    if np.min(Rover.rock_dists) < 10:
        return True
    else:
        return False
    
def rover_home_detected(Rover):
    home_pos_x = 99.7
    home_pos_y = 85.6
    distance = np.sqrt((home_pos_x - Rover.pos[0])**2 + \
                                        (home_pos_y - Rover.pos[1])**2)
    #print ('rover_home_detected = ' , distance)
    if distance < 5:
        return True
    else:
        return False

def rover_select_home_steer(Rover):
    home_pos_x = 99.7
    home_pos_y = 85.6
    
    angle_world_home = normalize_angle(np.arctan2(home_pos_y - Rover.pos[1], home_pos_x - Rover.pos[0]) * 180/np.pi) 
    angle_rover_home = angle_world_home - Rover.yaw  
    angle_clipped = np.clip(angle_rover_home, -15, 15)
    angle_filter = filter_steer_correction(angle_clipped)
    #print ('angle_world_home = ' , angle_world_home, 'angle_rover_home = ' , angle_rover_home ,' angle_clipped = ' , angle_clipped , 'angle_filter = ', angle_filter)
    return angle_filter    

def rover_close_to_home_check(Rover):
    home_pos_x = 99.7
    home_pos_y = 85.6

    distance = np.sqrt((home_pos_x - Rover.pos[0])**2 + \
                                        (home_pos_y - Rover.pos[1])**2)
    #print ('rover_close_to_home_check = ' , distance)
    if distance < 1:
        return True
    else:
        return False

# Normalize the angle between 0 and 360
def normalize_angle(angle):
    result = angle
    while(result <0):
        result+=360
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

# Decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.mode == 'forward':
            if Rover.samples_collected == 5:
                Rover.mode = 'gohome'
            # Check if we are stuck
            if rover_is_position_stuck_check(Rover):
                Rover.mode = 'stuck'
            else:
                if rover_rock_detected(Rover) :
                    if Rover.vel > 0.5:
                        rover_stop(Rover)
                    else:
                        rover_set_velocity(Rover,0.1)
                        # Set the navigable angle
                        if (rover_rock_reacheable_check(Rover)):
                            Rover.steer = rover_select_rock_steer(Rover)
                    
                        if rover_close_to_rock_check(Rover):
                            Rover.mode = 'pickuprock'
                # Check if there is navigable terrain ahead       
                elif navigable_terrain_check(Rover,threshold=Rover.stop_forward):
                    
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
                    Rover.mode = 'forward'
                    # Reset the stuck timer
                    Rover.stuck_position_time = time.time()
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
        elif Rover.mode == 'gohome':
            if rover_is_position_stuck_check(Rover):
                Rover.mode = 'stuck'
            else:
                if rover_home_detected(Rover):
                    if Rover.vel > 0.5:
                            rover_stop(Rover)
                    else:
                        rover_set_velocity(Rover,0.1)
                        # Set the navigable angle
                        Rover.steer = rover_select_home_steer(Rover)
                    
                        if rover_close_to_home_check(Rover):
                            Rover.mode = 'gameover'    
                elif navigable_terrain_check(Rover,threshold=Rover.stop_forward):
                    rover_set_velocity(Rover,Rover.throttle_set)
                    # Set the navigable angle
                    Rover.steer = rover_select_navigation_steer(Rover)
                else:
                    # There is not navigable terrain ahead, enter stop mode
                    Rover.mode = 'stop'
        elif Rover.mode == 'gameover':
            print("Game over")
            rover_stop(Rover)

                        
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        
    print("Rover.mode = ",Rover.mode," Rover.throttle = ",Rover.throttle," Rover.brake = ", Rover.brake," Rover.vel = ",Rover.vel," Rover.steer = ",Rover.steer)
    
    #Rover.throttle = 0
    #Rover.brake = 0
    #Rover.steer = 0
    
    return Rover