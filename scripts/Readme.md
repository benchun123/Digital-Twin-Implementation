# Isaac Sim with Demonstration
update 20230328

terminal 1:
roscore

## How to run uArm
terminal 2: (launch simulation)
source ~/script/isaac.sh 
cd benchun/IsaacProjects/Demonstrator/
source devel/setup.bash
python scripts/ifl_uArm_bk.py 

terminal 3: (publish ros actionlib)
source ~/script/isaac.sh 
cd benchun/IsaacProjects/Demonstrator/
source devel/setup.bash 
python scripts/uarm/uarm_client_test.py 


Pay attention: 
1. to use pump, it should be close to the cuboid surface, otherwise it does not work
for example, for the cuboid with position ([0.2, 0.0, 0.025]) and scale ([1.0, 1.0, 1.0]), I think it is 0.05m. To grasp it, the pump position should be (0.20, 0.0, 0.04), if z=0.05, it will fail. 
2. for uArm movement in Simulation, it can move very fast or slow, need more state check
3. for uArm movement, how to check if it reach goal, one is timesleep, just wait 6s and do next action even if does not reach the goal, the other is to check postion, use euclidean distance, this way need to consider the uArm limitation in case loop lock. 

remaining problem: 
1. How to check uArm limitation?
2. for uArm move, now is check the position for 6s, if it doesn't reach, continue to next step. (because of limitation, it doesnot reach)
3. other rostopics or rosservice?



## How to run Slider
terminal 2: (launch simulation)
source ~/script/isaac.sh 
cd benchun/IsaacProjects/Demonstrator/
source devel/setup.bash
python scripts/ifl_slider_bk.py 

terminal 3: (publish ros actionlib)
source ~/script/isaac.sh 
cd benchun/IsaacProjects/Demonstrator/
source devel/setup.bash 
python scripts/slider/slider_client_test.py 

remaining problem: 
1. do we need to set slider speed?
2. slider is just the wagen or together with uArm?


## How to run Conveyor
terminal 2: (launch simulation)
source ~/script/isaac.sh 
cd benchun/IsaacProjects/Demonstrator/
source devel/setup.bash
python scripts/ifl_conveyor_bk.py 

terminal 3: (publish ros actionlib)
source ~/script/isaac.sh 
cd benchun/IsaacProjects/Demonstrator/
source devel/setup.bash 
python scripts/conveyor/conveyor_client_test.py 

remaining problem: 
1. we simulate a joint pose to replace conveyor pose, but the timestamp and scale is not clear ... (the idea comes from slider, but control command is different)
2. other rostopics or rosservice?


## How to run Turtlebot

1. ifl turtlebot model
modify everything in .xacro, use following command to convert the model to .urdf, then, import urdf to isaac to get .usd, add ROS node to .usd. 

rosrun xacro xacro -o turtlebot3_burger_ifl.urdf turtlebot3_burger_ifl.urdf.xacro

attention: when import urdf to isaac, unclick "Fix Base Link" and "Create Physics Scene", change Stage Units Per Meter to "100", switch Joint Drive Type to "Velocity"

Test turtlebot model and movement with "Isaac Example -> Bin Filling Test"

## add json file
the file looks like: 
    "conveyor_1": {
        "robot_type" : "Conveyor",
        "world_prim" : "/World/conveyor_1",
        "robot_name" : "my_conveyor",
        "init_position": [0.0, 0.20, 0.0],
        "init_rotation" : [0.0, 0.0, 1.57],
        "init_scale" : [0.001, 0.001, 0.001]
    }

"robot_type" : search the model, limit to "Conveyor", "uArm", "Slider", ...
"world_prim" : name that add to stage: "/World/conveyor_1"
"robot_name" : rostopic namespace, should be different 
"init_position": [0.0, 0.20, 0.0],
"init_rotation" : [0.0, 0.0, 1.57],
"init_scale" : [0.001, 0.001, 0.001]