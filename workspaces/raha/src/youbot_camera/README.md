youbot_camera
=============

Provides a node which calibrates the camera position based on an AR tag fixed to the base.
Also has a launch file for bringing up the camera.

Bringing up the camera
----------------------

```> roslaunch youbot_camera openni.launch```

This will call the openni.launch in the openni_launch package with the camera name xtion.
It will also run a script which will keep the XnSensorServer running until the launch is cancelled.
This is important because the robot has been known to lock up when openni is running without the XnSensorServer.
  
Make sure the camera drivers are installed and the environmental variable XN_SERVER_LOCATION is defined.
If that variable is not defined, find the path to XnSensorServer, which should be in the bin of the installed drivers.
Then put that path into the variable like so:

```> echo XN_SERVER_LOCATION=path```

If you want that variable to be automatically defined in every new terminal you run, add that line to your .bashrc file.
Make sure that the XnSensorServer has global execute permissions.
If it does not, give it such permissions by going to its location and running:

```> sudo chmod a+x XnSensorServer```

Running the calibrator
----------------------

Make sure the ar_pose package is installed. 
If you want a groovy version, one can be found here: https://github.com/mjcarroll/ccny_vision/tree/groovy-devel
Change the camera topics in ar_pose_single.h from camera/stuff to xtion/stuff. (This should really be parameterized)
Run ar_pose:

```> roslaunch youbot_camera ar_pose.launch```

This will launch ar_pose with the rail marker. 
Make sure the marker is within the view of the camera. See the ar_pose documentation.
When ready, calibrate the camera's pose by running the calibrator, like so:

```> roslaunch youbot_camera calibrator.launch```

This will overwrite a line in the youbot's urdf which specifies the gripper_palm_link to xtion_camera frame.
To see the result of the calibration, restart the joint state publisher.
