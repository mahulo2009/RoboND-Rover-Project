## Project: Search and Sample Return

---

### Notebook Analysis


#### Obstacles and rock sample indentification

The perspect_transform function has been modified to include the perspective transformation of an image with all values ​​to one. This mask will then be used to define the inverse of the terrain, that is, the obstacles.

```python
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))

    return warped,mask
```

The color_thresh function has been modified to manage a threshold with maximum and minimum range for each RGB channel. This change allows to reuse this function for both terrain and rocks.

```python
def color_thresh(img, rgb_thresh=(0,0,0,0,0,0)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) & (img[:,:,0] <= rgb_thresh[1]) \
                    & (img[:,:,1] > rgb_thresh[2]) & (img[:,:,1] <= rgb_thresh[3]) \
                    & (img[:,:,2] > rgb_thresh[4]) & (img[:,:,2] <= rgb_thresh[5])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
```

#### Creating a worldmap

After defining the source and destination points in the image, we apply the perspective transformation. This function returns the converted image as well as the transformation of an image with all its values ​​to one, to be used to define the non-navigable region, ie, obstacles.

```python
    warped,mask = perspect_transform(img, source, destination)
```

The warped image is applied a color threshold to detect navigable terrain, rocks, and obstacles such as the inverse of navigable terrain.

```python
    terrain_threshed = color_thresh(warped,rgb_thresh=(160,255,160,255,160,255))
    obstacles_threshed = np.absolute(np.float32(terrain_threshed)-1)*mask
    rock_threshed = color_thresh(warped,rgb_thresh=(100,255,100,255,0,70))
```

An image is constructed with the result, using each of the RGB channels to visualize what has been identified as obstacle, as rock, or as navigable terrain.

```python
    output_image[0:img.shape[0], img.shape[1]:,0] = obstacles_threshed*255
    output_image[0:img.shape[0], img.shape[1]:,1] = rock_threshed*255
    output_image[0:img.shape[0], img.shape[1]:,2] = terrain_threshed*255
```

We convert from image coordinates to rover coordinates.

```python
    xpix, ypix = rover_coords(terrain_threshed)
```

Finally, taking into account the position and orientation of the rover in the world, we convert from rover coordinates into coordinates in the world.

```python
    scale = 10.0
    x_world, y_world = pix_to_world(xpix, ypix,
                                    float(data.xpos[data.count]),
                                    float(data.ypos[data.count])
                                    float(data.yaw[data.count])
                                    data.worldmap.shape[0],
                                    scale)
    data.worldmap[y_world, x_world,2] += 10
```

[![Navigation video](http://img.youtube.com/vi/q6FgESy9jy0/0.jpg)](http://www.youtube.com/watch?v=q6FgESy9jy0 "Navigation video - Clickt to Watch")

### Autonomous Navigation and Mapping

In autonomous navigation the rover will travel as much of the terrain as possible, picking up the rocks that are found in its path and returning to the starting point when all the rocks have been collected.

Several strategies have been used to achieve this goal; which will then be explained in more detail, but which can be summarized in:

* The rover will always move along the wall, to its right. In this way it will travel all over the world, without using sophisticated methods of navigation.
* The strategy to navigate is based on the average of the angles, in polar coordinates, of the navigable terrain. On some occasions this strategy makes the rover of laps in circles. To avoid this, when standard deviation is very high, normally in open spaces where navigable terrain exists in many directions, a random spin is introduced to the navigation direction.
* In case it does not have navigable terrain in front, it will turn around until it finds navigable terrain again. This condition is useful when it  is in a cul-de-sac.
* At certain times, the rover is stuck between obstacles. A strategy has been implemented, based on the choice of a random unstuck angle, which allows to recover the state of navigation again.
* During the navigation the rover is picking up the rocks that it finds in its passage. In addition, when it finishs collecting all the rocks, it must return to the starting point. The solution chosen does not differentiate between the way to collect the rock or go to the starting point: the rover navigates and when it is near a target it slows down and heads towards it. The goal can be either to take the rock or to go to the starting point.

#### Perception

#### Decision

The decision algorithm has been implemented as a state machine. Possible states are: forward, stop, stuck, pickuprock, finishpickuprock and gameover.

The following table shows a description of each state.

State | Description
------------ | -------------
forward | The rover moves along the right wall.
stop | The rover slows down and then turns on itself until it finds navigable terrain again.
stuck | The rover reduces speed. An angle to exit the stuck is chosen randomly.
pickuprock | Stop the rover and collect the rock.
finishpickuprock | Check if the rock was collected.
gameover | The rover stops at the starting point.

From the forward mode we can switch to the following modes: stuck, stop, pickuprock and gameover.

* From forward to stuck: if the rover is speed zero for five seconds.
* From forward to stop: if we do not have navigable terrain in front.
* From forward to pickuprock: if we are near a rock.
* From forward to gameover: if we are near to starting point and we have alreay recollected all the rocks.

##### Forward mode

In forward mode the rover checks: if it is stuck, if its speed is zero for five seconds; if it has detected a target: either rock or starting point after collecting all rocks; if it has navigable terrain in front, in which case it continues moving; or if it does not have navegable terrain, in which case it goes into stop mode.

```python
# Get reference to the rover controller
rover = RoverController.RoverController(Rover)
# Select the strategy to go to a target: pick up rocks or go home.
if Rover.samples_collected == 6:
    target = Strategy.StrategyHome(Rover)
else:
    target = Strategy.StrategyRock(Rover)

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
```

###### Detect we are stuck

To check if the rover is stuck it is seen if its speed is close to zero for 5 seconds, taking into account that we are in forward mode.

```python
def is_stuck(self):
    # Threshold to identify if the rover is stuck
    stuck_time_threshold = 5
    
    if math.fabs(self.rover.vel) < 0.1:
        #Check for how long I have not been moving. 
        stuck_time = time.time() - self.rover.stuck_position_time
    
        if stuck_time > stuck_time_threshold:
            return True
        else:
            return False
    else:
        # Reset the timer
        self.rover.stuck_position_time = time.time()
```

###### Detect we are near target

The goal can be a rock or the starting point. In decision_step no distinction is made between these two scenarios, the strategy is modified in case we have already collected all the rocks. When a target is detected, the speed is reduced and the rover is directed towards the target.

###### Detect rover is near rock

```python
def is_detected(self):
    distance = np.sqrt((self.home_pos_x - self.rover.pos[0])**2 + (self.home_pos_y - self.rover.pos[1])**2)
    if distance < 5:
        return True
    else:
        return False
```

###### Detect rover is near to the starting point

```python
def is_detected(self):
    distance = np.sqrt((self.home_pos_x - self.rover.pos[0])**2 + (self.home_pos_y - self.rover.pos[1])**2)
    if distance < 5:
        return True
    else:
        return False
```

###### Detect target is reacheable 

It may happen that a rock is detected but that it is not reacheable from the position where the rover is located because there are obstacles in between.

```python
def is_reacheable(self):
    if (len (self.rover.rock_angles)!=0):
        rock_angles_mean = np.mean(self.rover.rock_angles)
        navigable_angle_index_ = self.rover.nav_angles < rock_angles_mean
        nav_angles = self.rover.nav_angles[navigable_angle_index_]
        if len(nav_angles) > 0:
            return True
        else: 
            return False
    else:
        return False
```

###### Filter navigable terrain poinst father than 4 meters

During navigation the rover only uses points no more than four meters.

```python
def filter_navigable_angles(Rover):
    distance_thredshold = 40
    index_distance_faraway = Rover.nav_dists < distance_thredshold
    nav_angles = Rover.nav_angles[index_distance_faraway]
    return nav_angles
```

###### Check if there is navigable terrain ahead

To verify that we have navigable terrain we can count the navigable points no more than four meters away.

```python
def is_navigable_terrain(self,threshold):
    #Filter navigable terrain points farther than 4 meters
    nav_angles = filter_navigable_angles(self.rover)
    # The number of points ahead is bigger than threshold
    if len(nav_angles) >= threshold:
        return True
    else: 
        return False
```

###### Select the steer angle to navigate

The rover will always move along the wall, to its right. Therefore, an offset of -11 degrees is applied to the mean of the navigation points, in polar coordinates. If the standard deviation is very high, a random factor is applied to the navigation angle, to avoid moving in circles. Finally, if the correction is small it does not apply, to avoid that the rover oscillates.

```python
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
        stare_angle_random_factor = random.randrange(-1,1)
    # Calculate the steer angle
    steer_angle = nav_angles_mean + stare_angle_offset * stare_angle_random_factor
    # Clip then angle between -15 and 15. The values the rover accepts. 
    steer_clipped = np.clip(steer_angle, -15, 15)
    #Return the filter steer angle
    return filter_steer_correction(steer_clipped)
```

##### Stuck mode

In stuck mode the speed is reduced, a random angle is selected to exit the stuck and it is attempted to advance in this direction.

```python
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
```

##### Stop mode

In stop mode speed is reduced and the rover rotates on itself until it can see navigable terrain again.

```python
elif Rover.mode == 'stop':
    # Reduce velocity if we are still moving 
    if Rover.vel > 0.2:
        rover.stop()
    else:
        # Twist the rover until navigable terrain ahead again
        rover.twist()
        if rover.is_navigable_terrain(threshold=Rover.go_forward) :
            Rover.mode = 'forward'
```

##### Pickup Rock mode

In pickup mode the rover stops and picks up the rock. When it has finished picking up the rock, go to finishpickuprock mode.

```python
elif Rover.mode == 'pickuprock':
    # Stop the rover to pick up the rock
    rover.stop()
    # Pick up the rock
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    else:
        # Move to detect if the rock has been picked up                
        Rover.mode = 'finishpickuprock'
```

##### Finish Pickup Rock mode

In finishpickrock mode it is checked if the rover finished picking up the rock and starts sailing again.

```python
elif Rover.mode == 'finishpickuprock':
    if  not Rover.picking_up:
        #Reset stuck time and go forward
        Rover.stuck_position_time = time.time() 
        Rover.mode = 'forward'
```

##### Game over mode

In gameover mode the rover stops.

```python
elif Rover.mode == 'gameover':
    print("Game over")
    rover.stop()
```
