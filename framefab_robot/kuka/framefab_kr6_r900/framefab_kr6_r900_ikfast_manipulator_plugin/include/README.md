Following the moveit_ikfast instruction [here](http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html), we can generate this ikfast plugin package.

But to make it compatible with framefab_kr6_r900_descartes package, we have to manually change the following:

Line 118-129, make these member variable `protected` instead of `private`

Line 277, make these member functions `protected`.
