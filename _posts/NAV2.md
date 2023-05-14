1. Launch turtlebot with gazebo:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Launch Turtule bot with a saved map:
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=<path_to_map.yaml> use_sim_time:=True
```

# UNDERSTAND THE NAV2 STACK
## 1. GLOBAL PLANNER, LOCAL PLANNER, COSTMAPS
### 1.1. Global Planner
- Global planner is responsible to generate a path initial position to goal.
- To Generate a path, the global planner needs two things:
  - 1. A Map of the environment:
     
     ![environment map](../assets/images/Nav2/my_map.png)
