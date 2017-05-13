##kuka_kr6r900sixx_with_hotend

- Author: 		Yijiang Huang (MIT)
- Email: 			yijiangh@mit.edu
- Created on: 5-10-2017
- Status:			Tested in Rviv env, plan to test on real robot June 2017.
- Devel Status:	Tested on ROS-indigo (indigo-devel for depended package as well).

**Catkin Dependencies:**
- [industrial_core](http://wiki.ros.org/industrial_core)
- [kuka-experiemental](http://wiki.ros.org/kuka_experimental)

### result SRDF overview

<robot name="kuka_kr6r900sixx_with_hotend">

    <group name="manipulator">
        <chain base_link="arm_base_link" tip_link="tool_tip" />
    </group>

    <virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="arm_base_link" />

		<!-- omit disable collision -->
</robot>

### Test & Launch

#### Test xacro file
run: `roslaunch kr6r900sixx_with_hotend test_kr6r900sixx_with_hotend.launch`

#### Test moveit config in Rviz
run: `roslaunch kr6r900sixx_with_hotend_moveit_config demo.launch`

### Reference:

#### XACRO & URDF creation: 

Based on knowledge learnt from [kuka_kr120_robot](https://github.com/JeroenDM/kuka_kr120_robot/tree/dd47dedebe0baf97b16ac3b67f1b5148e9b3cf05), used in [descartes_tutorials](https://github.com/JeroenDM/descartes_tutorials/tree/ef8819baa692e37b1325f53a59fb3d5213baa15c) by @JeroenDM.

#### Moveit setup assistant:

[youtube video tutorial from Sachin Chitta](https://www.youtube.com/watch?v=asg-thB3mwA)

For fixed-joint end effector (e.g. 3D printing hotend, welding torch etc, not gripper-like acturated effector), when add Planning Groups:

1. add manipulator group
	- Add Group, name = 'manipulator', kinematic solver = 'KDLKinematicsPlugin'
	- Add **Kin. Chain**, this method is generally preferred over Add Joints or Add Links assign base_link and tip_link (e.g. link_6, tool0, or your robot's equivalent (in this case *tool_tip*))

2. **No need to create end-effctor group or add end effector in following step**, as the robot will take this as a part of its own fixed link (fixed relative to tool0).

Refer to [ros tutorial](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot).

#### Known issue with IKFast & openrave

**Error**: `TypeError: symbolic boolean expression has no truth value`

**Reference**: [moveit! google group](https://groups.google.com/forum/#!topic/moveit-users/5tP9gXY_siM) and [openrave issue#470](https://github.com/rdiankov/openrave/issues/470)

**Solution**: For ubuntu 14.04, default sympy is 0.7.4, please use `pip install sympy==0.7.1` to downgrade to `0.7.1`. (type in `pip list | grep sympy` to check)

#### SRDF planning group syntax (get from SDRF file commment)

 - GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc

 - LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included
 
 - JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included

 - CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group

 - SUBGROUPS: Groups can also be formed by referencing to already defined group names

