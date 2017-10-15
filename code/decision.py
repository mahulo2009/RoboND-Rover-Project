import time
import random
import Strategy,RoverController

# Decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Get reference to the rover controller
        rover = RoverController.RoverController(Rover)
        # Select the strategy to go to a target: pick up rocks or go home.
        if Rover.samples_collected == 6:
            target = Strategy.StrategyHome(Rover)
        else:
            target = Strategy.StrategyRock(Rover)
        
        # Mode move forward        
        if Rover.mode == 'forward':
            # Check if we are stuck
            if rover.is_stuck():
                Rover.mode = 'stuck'
            else:
                # Target detected reduce velocity and go for it
                if target.is_detected() :
                    if Rover.vel > 0.5:
                        rover.stop()
                    else:
                        rover.set_velocity(0.1)
                        # If target is reacheable, no obstacles between the rover and target. 
                        if (target.is_reacheable()):
                            Rover.steer = target.select_steer()
                        # If we are close enough to the target    
                        if target.is_close():
                            if Rover.samples_collected == 6:
                                Rover.mode = 'gameover'                                
                            else:
                                Rover.mode = 'pickuprock'
                # Check if there is navigable terrain ahead       
                elif rover.is_navigable_terrain(threshold=Rover.stop_forward):
                    # Set the navigable velocity
                    rover.set_velocity(Rover.throttle_set)
                    # Set the navigable angle
                    Rover.steer = rover.select_navigation_steer()
                else:
                    # There is not navigable terrain ahead, enter stop mode
                    Rover.mode = 'stop'
        elif Rover.mode == 'stop':
            # Reduce velocity if we are still moving 
            if Rover.vel > 0.2:
                rover.stop()
            else:
                # Twist the rover until navigable terrain ahead again
                rover.twist()
                if rover.is_navigable_terrain(threshold=Rover.go_forward) :
                    Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            # Reduce velocity if we are still moving
            if Rover.vel > 0.2:
                rover.stop()
            else:
                # Twist the rover in the  unstuck  direction
                rover.select_unstuk_yaw()
                # Go forward in the unstuck direction
                if rover.is_unstuk_yaw_reached():
                    # Reset the stuck timer
                    Rover.stuck_position_time = time.time()
                    # Select the next unstuck direction
                    Rover.unstuck_yaw=random.uniform(0,360)
                    Rover.mode = 'forward'
        elif Rover.mode == 'pickuprock':                       
            # Stop the rover to pick up the rock
            rover.stop()
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
        elif Rover.mode == 'gameover':
            print("Game over")
            rover.stop()
                        
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        
    print("Rover.mode = ",Rover.mode," Rover.throttle = ",Rover.throttle," Rover.brake = ", Rover.brake," Rover.vel = ",Rover.vel," Rover.steer = ",Rover.steer)
    
    #Rover.throttle = 0
    #Rover.brake = 0
    #Rover.steer = 0
    
    return Rover