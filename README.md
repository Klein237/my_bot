# navbot
Here ROS development for the autonomous navigation with nav2

## create your workspace
```
mkdir -p ws/src
```

## How to launch the simu

After build and source the package: 
### launch the autonomous navigation with slam and map
```
ros2 launch my_bot nav1.launch.py
```
### launch to see the robot used to the gps navigation, don't forget to use Rviz

```
ros2 launch my_bot view.launch.py
```
### launch the navigation by gps with nav2 package ( don't work)
```
ros2 launch my_bot gnav.launch.py
```
### others launch
```
ros2 launch my_bot simfr.launch.py
ros2 launch my_bot launch_sim.launch.py
```
* we have also the node create to simulate a vitual gps but to work you a json file with the gps coordonate 
```
ros2 run my_bot vitual_gps.py
```
### client node to save the coordinates, make sure that nav1.launch.py run also set the right path of the json file in your workspace
```
ros2 launch my_bot save_coordinates_client.py
```
### others nodes, make sure that nav1.launch.py run also set the right path of the json file in your workspace
```
ros2 run my_bot nav_through.py
ros2 run my_bot waypoint_follown.py
```
