# __Related Projects and Tutorials Usage__
## &sect; ROS1 driver usage
> 
> After the user has set up the ROS1 environment and built the TM driver based on the specific workspace, please enter your workspace `<workspace>` by launching the terminal, and remember to make the workspace visible to ROS.
>
>
> ```bash
> source /opt/ros/noetic/setup.bash
> cd <workspace>
> source ./devel/setup.bash
> ```
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running. 
> 
> Then, run the driver to test whether the complete communication interface is properly working with TM Robot by typing 
>
>```bash
> rosrun tm_driver tm_driver <robot_ip_address>
>```
> Example :``rosrun tm_driver tm_driver 192.168.10.2``, if the <robot_ip_address> is 192.168.10.2
>
> Now, the user can use a new terminal to run each ROS node or command, but don't forget to source the correct setup shell files as starting a new terminal.
> Note: When you finish executing your developed scripts or motion commands through the TM ROS driver connection, press __CTRL + C__ in all terminal windows to shut everything down.

## &sect; Usage with MoveIt
>
> See [Moveit tutorial](https://moveit.ros.org/install/) to install the MoveIt packages.<br/>
> ```bash
> sudo apt install ros-noetic-moveit
> ```
>
> Then, use the following command to install ROS-Industrial packages on Ubuntu, working with ROS Noetic:
> ```bash
> sudo apt-get install ros-noetic-industrial-robot-simulator
> ```
>
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.<br/>
>
> * To bring up the MoveIt environment in simulation mode with the virtual TM Robot, by typing<br/>
>
>
> ```bash
> roslaunch <tm_robot_type>_moveit_config <tm_robot_type>_moveit_planning_execution.launch sim:=True
> ```
>
>> The prefix `<tm_robot_type>` means the TM Robot type, available for tm5s, tm7s, tm12s, tm14s, and tm25s models.
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing
> ```bash
> roslaunch tm12s_moveit_config tm12s_moveit_planning_execution.launch sim:=True
> ```
>
> * The user can also manipulate the real TM Robot to run, by typing<br/>
>
> ```bash
> roslaunch <tm_robot_type>_moveit_config <tm_robot_type>_moveit_planning_execution.launch sim:=False robot_ip:=<robot_ip_address>
> ```
> :warning:[CAUTION] This demo will let the real TM Robot move, please be careful. If the user are a beginner or unfamiliar with the arm movement path, it is recommended that the user place your hand on the big red emergency _Stick Stop Button_ at any time, and press the button appropriately in the event of any accident that may occur.<br/>
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing<br/>
>
> ```bash
> roslaunch tm12s_moveit_config tm12s_moveit_planning_execution.launch sim:=False robot_ip:=<robot_ip_address>
> ```
>
>> The parameter `<robot_ip_address>` means the IP address of the TM Robot.<br/>
>
> Note: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>
> :bookmark_tabs: Note1: There are several built-in TM Robot nominal robot model settings, available for TM5S, TM7S, TM12S, TM14S, and TM25S models.<br/>
> :bookmark_tabs: Note2: In MoveIt planning_context.launch, TM Robot set the default to read the Xacro file, such as _TM5S_ model, to read the file _tm5s.urdf.xacro_ into robot_description or such as _TM12S_ model, to read the file _tm12s.urdf.xacro_ into robot_description. If the user wants to use the specific model parameters instead of the nominal model to control the robot, please go back to the section __6. Generate your TM Robot-Specific Kinematics Parameters Files__ to modify the Xacro file.<br/>
> :bookmark_tabs: Note3: __Running two tm ros drivers at the same IP address is not allowed.__ Since the tm driver node has been written into the moveit launch file, there is no need to execute _rosrun tm_driver tm_driver <robot_ip_address>_.<br/>

> __Usage with Gazebo Simulation__ 
>
> See [Gazebo tutorial](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) to install the Gazebo packages.<br/>
>> Then, install the other joint_trajectory_controller plugin: <br/>
`` sudo apt-get install ros-noetic-joint-trajectory-controller``<br/>
`` sudo apt-get install ros-noetic-rqt-joint-trajectory-controller``<br/>
>
> The tm_gazebo package contains the Xacro model files to simulate the TM Robot in Gazebo.
>
> There are several built-in launch files that can be used to start the TM Robot simulated robot using the nominal Xacro robot model settings in Gazebo.
> The common command's form to bring up the TM simulated robot in Gazebo as follows: 
>
> ```bash
> roslaunch tm_gazebo <tm_robot_type>_gazebo.launch
> ```
>
> The prefix `<tm_robot_type>` means the TM Robot type, available for the tm5s, tm7s, tm12s, tm14s, and tm25s models.<br/>
> For the TM12s Robot, simply replace the prefix accordingly to tm12s and type "``roslaunch tm_gazebo tm12s_gazebo.launch``".<br/>
> :bookmark_tabs: Note1: If your real Robot is a TM5S, in the above example, you should type tm5s_gazebo.launch.<br/>
> :bookmark_tabs: Note2: If the user needs to improve end-point simulation accuracy, please refer to the following section __Take generating a new Xacro file as an example__ of chapter 6 to modify the Xacro file.<br/>

> __Using Moveit! with Gazebo Simulator__
>
>  You can also use MoveIt! to control the simulated robot which is configured to run alongside Gazebo.
> 
> 1. Launch the Gazebo simulation and load the ros_control controllers:
> ```bash
> roslaunch tm_gazebo <tm_robot_type>_gazebo.launch
> ```
> After the Gazebo simulator is running, proceed to the next command to launch moveit!.
> 
> 2. Launch the combined of moveit! and Gazebo to allow motion planning plugin run:
> ```bash
> roslaunch <tm_robot_type>_moveit_config <tm_robot_type>_moveit_planning_execution_gazebo.launch
> ```
> Taking the TM12S robot as an example, use the commands introduced above:
> Note: If you have started some executable programs with ROS commands in some terminal windows, it is recommended that you close them and then execute the following commands.
> 1. To open the terminal 1: Running with Gazebo<br/>
``source /opt/ros/noetic/setup.bash``<br/>
``cd <workspace>``<br/>
``source ./devel/setup.bash``<br/>
``roslaunch tm_gazebo tm12s_gazebo.launch``<br/>
> 
> 2. In a new terminal 2: Running with moveit!<br/>
``roslaunch tm12s_moveit_config tm12s_moveit_planning_execution_gazebo.launch``<br/>
>
> :bookmark_tabs: Note1: Remember to close all these executables when you no longer use them for Gazebo simulations.<br/>
> :bookmark_tabs: Note2: Sometimes when gzserver is not properly shut down with ROS or cannot run Gazebo again after shutting down, you can try to kill the corresponding process with the following command.<br/>
>>:bulb: **Tip**: To kill both the Gazebo server and Gazebo client executables.<br/>
>> ``sudo killall -9 gazebo gzserver gzclient``<br/>
>
<div> </div>

