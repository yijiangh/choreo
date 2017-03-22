## Overview:

This data set is an early-stage sequence result for testing and building Robotic Printing MPP.

I also attach the file ball.pwf, which is the input geometry for framefab before computation. It contains only a list of node, and their connection info (using the index of the node list). The extra info remained are for framefab computation initialization setup.

**Node example:**

    v 680.265930 619.621704 -296.658630
    v 732.258850 608.148987 -297.348328
    v 617.694092 564.329773 -296.927856
    v 623.799683 509.152924 -297.225220
    v 654.643250 478.069275 -295.272705
    v 697.632324 469.088837 -296.411896
    v 743.790588 490.274567 -297.143188
    v 765.171631 529.831360 -295.703247
    v 760.725952 573.934143 -295.63043

**Connection example:**

    l 1 2
    l 3 4

**Framefab Init info**

`p 1 31` indicates that the member connecting node[1] and node[31] is a `pillar` (members that support the model, not in original model, but have to be printed)

`c 29 30` indicates that the member connecting node[29] and node[30] is a `ceiling` (used for the graph cut algorithm in Framefab).

Indicates the the node[1] and node[2] in the node list above is connected.

**Object: Semi-sphere**

Contains **87** linear elements.

![](http://i.imgur.com/CK7ekE2.png)

## Data specification:

**INormal_x.txt** Contains the end effector orientation info for printing linear element **No.x**, a unit vector representation.

**IStart.txt** Contains a list (87 3D point in this case) of start positions (x,y,z) for each element. 

**IEnd,txt** Contains a list (87 3D point in this case) of end positions (x,y,z) for each element.

Thus linear element No.x can be assembled by connecting No.x point in IStart list and No.x point in IEnd point.

## Note:
### The coordinate description frame
The 3D frame in which these points are described **IS NOT** related to #BASE of the robot. In our algorithm, all the coordinates should be described w.r.t. the #BASE frame of the robot (the center of robot's axis 1, the base). Thus, a translation is needed to translate the model from current position (in this framefab-generated data to the heating bed position). And this is why I emphasis the importance of base pos and heating bed pos in our meeting at Mar/8/2017 (refer to our meeting sketch below).

![](https://files.slack.com/files-tmb/T2U6AT946-F4KJB1031-c3e4a3261e/robot_720.png)

### The end effector
I think you also have to specify the end effector for Moveit!. But I think you can leave it as original now. We can do it together later and I'll model it up. 

So now, just ---

Let the motion go red and not feasible at all hah :)