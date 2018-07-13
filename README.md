# Auto-Car-Cooperative-Path-Planning-02-Movement-Sequence-Planning
Python codes for robotics Multi-Vehicle-Movement-Sequence-Planning algorithm.
## What is this?
This is a Multi-Vehicle-Movement-Sequence-Planner. <br>
This method can do the Multi-Vehicle-Movement-Sequence-Planning based on the topological map.<br>
The output of this planner can be used in Multi-Vehicle-Trajectory-Planning.<br>
## How to coordinate the movement sequence of the vehicles? 
Movement Sequence Planning based on the **Pass Priority**.
### Why?
Any multi-vehicle conflict problem can be decomposed into multiple two-vehicle conflict problems.<br>
Each two-vehicle conflict is caused by two vehicles passing through the same road at the same time. <br>
The key to solving the problem is to coordinate the movement sequence, that is, the pass priority.<br>
### How?
1. List all Pass Priority combination. <br>
2. Evaluate and select optimal Pass Priority.<br>
3. Plan based on a pass priority.
## How to plan based on a pass priority?
1. Let the highest priority vehicle go first.<br>
    * Move other vehicles to yield region. <br>
2. Remove it from Pass Priority List.<br>
3. Loop<br>
<img src="https://github.com/ChenBohan/Robotics-Path-Planning-Multi-Vehicle-Movement-Sequence-Planning/blob/master/pic/move_to_yield_region.png" width = "70%" height = "70%" div align=center />

## Simulation
### Scenario
<img src="https://github.com/ChenBohan/Robotics-Path-Planning-Multi-Vehicle-Movement-Sequence-Planning/blob/master/pic/scenario.png" width = "40%" height = "40%" div align=center />

### All Solutions
<img src="https://github.com/ChenBohan/Robotics-Path-Planning-Multi-Vehicle-Movement-Sequence-Planning/blob/master/pic/all_solutions.png" width = "50%" height = "50%" div align=center />

### A Solution based on prority {A>C>B}
<img src="https://github.com/ChenBohan/Robotics-Path-Planning-Multi-Vehicle-Movement-Sequence-Planning/blob/master/pic/result.png" width = "50%" height = "50%" div align=center />








