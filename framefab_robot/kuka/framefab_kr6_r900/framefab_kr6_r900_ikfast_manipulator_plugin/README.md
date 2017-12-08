The generation of IKFast plugin requires openrave installation.

Unfortunately, ROS-kinetic does not have a publicly released openrave package.

Based on my experience, the easiest way to install openrave on Ubuntu 16.04 is `clone it and build from source`.

At Dec-8-2017, the following line in cmd FAILED to fetch openrave in Ubuntu 16.04.

```
sudo add-apt-repository ppa:openrave/release
sudo apt-get update
```

To build from source, run:

```
git clone https://github.com/rdiankov/openrave.git
cd openrave && mkdir build && cd build
cmake .. -DOSG_DIR=/usr/local/lib64/
make -j4
sudo make install
```

Notice that the openrave will be installed at this path:

```
/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_0_9
```

instead of 

```
/usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8
```

because we are doing source installation.

Here are some references:

[Most up-to-date IKFast Plugin generation guide from Moveit!](http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution/Prerequisites)

[Official openrave installation instruction](http://openrave.org/docs/latest_stable/install/#compile-from-sources)

[Installing OpenRave on Ubuntu 16.04: a tutorial from Stéphane Caron](https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html)

[ROS-I tutorial - Create a fastik solution - prerequisite](http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution/Prerequisites)

[ROS-I tutorial - Create a fastik solution](http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution)

[Discussion related to Descartes and IKFast on the main Descartes repo](https://github.com/ros-industrial-consortium/descartes/issues/204)


