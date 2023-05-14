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
    - A Map of the environment: Each cell is a binary represention if the cell it occupied or not. (occupied = 1, free = 0)
     
     ![environment map](../assets/images/Nav2/my_map.png)

    - A cost map:
        A cost map assigns a cost to each pixel (cell) of the map. The cost is a value that represents how difficult is to navigate throught that cell.

    ![cost map](../assets/images/Nav2/cost_map.png)

    By summing these two maps, we generate a global cost map. The global planner uses this global cost map to generate a path from the initial position to the goal.
