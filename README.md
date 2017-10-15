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


#### Perception

#### Decision







