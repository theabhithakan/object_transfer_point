# SETTING UP THE WORKSTATION
## Setting Kinect

Open skeleton tracking using Kinect
Log in to the Windows workstation.
Password is on the sticky note

Skeleton Tracking:
Start Delicode NI mate 2 and access it from the system tray icon.
Click on the icon and select Control Interface. 

In Setup
Default IP = 127.0  .0  .1

Port = 7000 Or whichever is in the server.js file

Start the service if not started already

Expand the Kinect for Windows option in the list
Select Skeleton tracking
In Skeleton tracking you can see the joints being tracked from the Kinect.
Select or Deselect Hands option according to the need

You can see the tracking on the Screen

**NOTE: Make sure that the Origin is set to Sensor**

Logging Kinect data to the Ubuntu workstation.
Run Windows command line
Press Win+R key, input ```cmd``` in the box and hit Enter.

Navigate to the folder
C:\Users\trina\Desktop\hiro_ws\websock-osc-master
```
cd Desktop\hiro_ws\websock-osc-master
```

And open this service
```
node server.js
```

This will enable the sharing of data with the Ubuntu workstation


# NOW MOVE OVER TO THE UBUNTU WORKSTATION

Make sure Baxtett is ON and ssh into the robot using
```
ssh trina@trina-server
```

To enable robot run the following command
```
rosrun baxter_tools enable_robot.py -e
```

To disable the robot run this command
```
rosrun baxter_tools enable_robot.py -d
```

Enable hands by
```
roslaunch ebolabot_bringup hands.launch
```


We will be working on the handover package of the hmc_ws
Navigate to the necessary folder and work on your part.


NOTE:
We are going to use ROS and Python heavily, so make sure that you learn it in the next couple of weeks
