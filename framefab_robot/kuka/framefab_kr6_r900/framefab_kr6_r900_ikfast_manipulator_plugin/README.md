### OpenRave installation

The generating IKFast plugin requires openrave installation.

Unfortunately, ROS-kinetic does not have a publicly released openrave package.

Based on my experience, the easiest way to install openrave on Ubuntu 16.04 is `clone it and build from source`.

At Dec-8-2017, the following line in cmd FAILED to fetch openrave in Ubuntu 16.04.

```
sudo add-apt-repository ppa:openrave/release
sudo apt-get update
```

To build from source, in the dir that you want to place openrave (better not be catkin_ws/src) run the following in terminal (based on my experience, we shouldn't need to install any extra package):

```
git clone https://github.com/rdiankov/openrave.git
cd openrave && mkdir build && cd build
cmake .. -DOSG_DIR=/usr/local/lib64/
make -j4
sudo make install
```

Notice that the openrave will be installed at this path (by the cmake option `-DOSG_DIR=/usr/local/lib64/`):

```
/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_0_9
```

instead of 

```
/usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8
```

To test your installation:

```
cd /usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_0_9

python ikfast.py --help
```

Here are some references:

[Most up-to-date IKFast Plugin generation guide from Moveit!](http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution/Prerequisites)

[Official openrave installation instruction](http://openrave.org/docs/latest_stable/install/#compile-from-sources)

[Installing OpenRave on Ubuntu 16.04: a tutorial from Stéphane Caron](https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html)

[ROS-I tutorial - Create a ikfast solution - prerequisite](http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution/Prerequisites)

[ROS-I tutorial - Create a ikfast solution](http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution)

[Discussion related to Descartes and IKFast on the main Descartes repo](https://github.com/ros-industrial-consortium/descartes/issues/204)

### Procedure

Follow [ikfast moveit! tutorial](http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html).

### Troubleshotting

#### bash: /usr/bin/openrave-robot.py: No such file or directory

If you are following ikfast tutorial [here](http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html), the compiled path is changed to /usr/local/bin/openrave... because we set the cmake option `/usr/local/lib64/`. Simply replace the path to `/usr/local/bin/openrave-robot.py` will fix the problem.

#### `failed to find an OpenRAVE viewer.` if you run `openrave <your dae>.dae`

check the message when you cmake openrave. You might find the following warnings:

```
-- Could not find OpenSceneGraph v3+. Please install OSG (http://www.openscenegraph.org/projects/osg)
-- Detected SoQt/Coin3D GUI, making plugin
WARNING: Could not find ODE. Please install ODE (http://www.ode.org)
```

Installing these missing packages:

```
sudo apt-get install libqt4-dev libsoqt-dev-common libsoqt4-dev
```

, recompile, and reinstall will fix the problem.

#### could not import scipy.optimize.leastsq

```
sudo apt-get install python-numpy python-scipy python-sympy 
```

then recompile, reinstall will fix the problem.


