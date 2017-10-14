## Project: Search and Sample Return

---

### Notebook Analysis


#### Obstacles and rock sample indentification

```python
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))

    return warped,mask
```



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

```python
     # Let's create more images to add to the mosaic, first a warped image
    warped,mask = perspect_transform(img, source, destination)

    terrain_threshed = color_thresh(warped,rgb_thresh=(160,255,160,255,160,255))
    obstacles_threshed = np.absolute(np.float32(terrain_threshed)-1)*mask
    rock_threshed = color_thresh(warped,rgb_thresh=(100,255,100,255,0,70))


    # Add the warped image in the upper right hand corner
    output_image[0:img.shape[0], img.shape[1]:,0] = obstacles_threshed*255
    output_image[0:img.shape[0], img.shape[1]:,1] = rock_threshed*255
    output_image[0:img.shape[0], img.shape[1]:,2] = terrain_threshed*255

    scale = 10.0

    xpix, ypix = rover_coords(terrain_threshed)
    x_world, y_world = pix_to_world(xpix, ypix,
                                    float(data.xpos[data.count]),
                                    float(data.ypos[data.count])
                                    float(data.yaw[data.count])
                                    data.worldmap.shape[0],
                                    scale)

    xpix_obstacle, ypix_obstacle = rover_coords(obstacles_threshed)
    x_obstacle, y_obstacle = pix_to_world(xpix_obstacle, ypix_obstacle,
                                    float(data.xpos[data.count]),
                                    float(data.ypos[data.count]),
                                    float(data.yaw[data.count]),
                                    data.worldmap.shape[0],
                                    scale)

    xpix_rock, ypix_rock = rover_coords(rock_threshed)
    x_rock, y_rock = pix_to_world(xpix_rock, ypix_rock,
                                    float(data.xpos[data.count]),
                                    float(data.ypos[data.count]),
                                    float(data.yaw[data.count]),
                                    data.worldmap.shape[0],
                                    scale)


    data.worldmap[y_obstacle, x_obstacle,0] += 1
    data.worldmap[y_rock, x_rock,1] += 1
    data.worldmap[y_world, x_world,2] += 10
```

[![IMAGE ALT TEXT](http://img.youtube.com/vi/q6FgESy9jy0/0.jpg)]


### Autonomous Navigation and Mapping


#### Perception

#### Decision







