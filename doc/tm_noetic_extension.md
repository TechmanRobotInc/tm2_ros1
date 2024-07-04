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
> Then, run the driver to maintain the connection with the real TM Robot by typing 
>
>```bash
> rosrun tm_driver tm_driver <robot_ip_address>
>```
> Example :``rosrun tm_driver tm_driver 192.168.10.2``, if the <robot_ip_address> is 192.168.10.2
>
> Now, the user can use a new terminal to run each ROS node or command, but don't forget to source the correct setup shell files as starting a new terminal.

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
>> The prefix `<tm_robot_type>` means the TM Robot type, available for tm5s, tm7s, tm12s, tm14s and tm25s models.
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
> :warning:[CAUTION] This demo will let the real TM Robot move, please be careful.<br/>
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing<br/>
>
> ```bash
> roslaunch tm12s_moveit_config tm12s_moveit_planning_execution.launch sim:=False robot_ip:=<robot_ip_address>
> ```
>
>> The parameter `<robot_ip_address>` means the IP address of the TM Robot.<br/>
>
> :bookmark_tabs: Note1: There are several built-in TM Robot nominal robot model settings, available for TM5S, TM7S, TM12S, TM14S and TM25S models.<br/>
> :bookmark_tabs: Note2: In MoveIt planning_context.launch, TM Robot set the default to read the Xacro file, such as _TM5S_ model, to read the file _tm5s.urdf.xacro_ into robot_description or such as _TM12S_ model, to read the file _tm12s.urdf.xacro_ into robot_description. If the user wants to use the specific model parameters instead of the nominal model to control the robot, please go back to the section __6. Generate your TM Robot-Specific Kinematics Parameters Files__ to modify the Xacro file.<br/>

<div> </div>

