# baxter_tufts
 
## Introduction:
This package when ran launches gazebo world, a table and a specified block on the table via a python script. The user specifies the block's serial number(0 through 4) and the number of times to run the code. The joint_state is recorded into a rosbag. The naming convention for the rosbag file is:
>                     <baxter_TypeOfAction__model><Block number>__<Time stamp>.bag
example:
>                      baxter_pick_and_place__model4__2018-08-01-14-25-14.bag

## Run:
To run the code, the user runs the following in the commandline:
>                     roslaunch baxter_tufts TypeOfAction.launch run_info:="block_number num_of_run"
example:
>                     roslaunch baxter_tufts pick_and_place_tufts.launch run_info:="4 1"
 
where:
- baxter_tufts                -> the name of package
- TypeOfAction                -> They are several actions, such as "pick_and_place_tufts", push_different_mass
- TypeOfAction.launch -> the launch file for the gazebo world and the demo python script
- run_info                    -> run information which will contain the run arguments
- block_number                -> for this demo, it's from 0 to 4. With 0 being the heaviest and 4, the lightest
- num_of_run                  -> number of desired time the robot should pick block. Number should be greater than 

## Video demos:
### Pick and Place demo:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/GDXMXK_m64g/0.jpg)](https://www.youtube.com/watch?v=GDXMXK_m64g)

### Push Different Mass demo:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/sgkIpK8fFvA/0.jpg)](https://www.youtube.com/watch?v=sgkIpK8fFvA&feature=youtu.be)