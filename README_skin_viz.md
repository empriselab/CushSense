Skin Visualization Documentation 
==============

# I. Directory Diagram

Below is a tree representation of the wholearm_ws directory. It includes all the files that are relevant to the skin visualization; these files will be detailed further in Section III. It is important to note this directory is comprised of two workspaces: `wholearm_ws` which visualizes the taxels and `controller_ws` which visualizes the robot. 

```
  wholearm_ws
    ├── controller_ws
      ├── src
          ├── yui_bathing
            ├── config
            ├── launch
              ├── viz.launch
              ├── master_launch_testing.launch	
              ├── …
            ├── …
        ├── libada 
          ├── config
          ├── launch
          ├── …
        ├── …
      ├── setup.bash
      ├── …
  ├── wholearm_ws
    ├── src
      ├── wholearm_skin_ros
        ├── launch
            ├── testing.launch
            ├── vizual.launch
            ├── …
        ├── scripts
          ├── viz_pub_testing.py
          ├── viz_sub_testing.py
          ├── viz_sub.py
          ├── gen_json.py
          ├── robot.json
          ├── custom_taxel_pub.py
          ├── inference.py
          ├── …
      ├── user_study
        ├── record.sh
        ├── sample_user
          ├── bag_1.bag
          ├── bag_2.bag     
        ├── rosserial_noetic
        ├── rosserial_python
          ├── nodes
            ├── serial_node.py
            ├── …
          ├── …
        ├── …
      ├── …
    ├── …
  ├── …
```

In addition to the wholearm_ws directory, there is also a seperate file for the arduino that needs to be ran: `test_link_read.ino`. It is located in the `/home/Arduino/test_link_read` folder.

# II. Running the Visualization 

First, connect to the Arduino and open the Arduino IDE. Then pull up `test_link.read.ino` and upload sketch.

Then, to run the visualization run the following commands:

Terminal 1
In `wholearm_ws`:
```
cd controller_ws 
roscore
```

Terminal 2:
In `wholearm_ws`:
```
cd controller_ws
source devel/setup.bash
roslaunch yui_bathing viz.launch
```
If the robot is off, the failure message failed to load controller spawner will be outputted. Please stop the terminal and turn on the robot, then try again.

Ignore "joint state with name: "left_inner_finger_joint" was received but not found in URDF" that means robot connected successfully.

Terminal 3:
In `wholearm_ws`:
```
cd wholearm_ws
source devel/setup.bash
roslaunch wholearm_skin_ros vizual.launch
```

Terminal 4:
In `wholearm_ws`:
```
cd controller_ws
source devel/setup.bash
rviz
```

The result of these commands will be an rviz window. Within this window make sure to click `add` at the bottom left, and under the category `by display type` click `RobotModel` and then `OK`. The again click `add`, but this time click `By Topic` and click `taxel_markers` and then `OK`. Yui can be moved by going to the [Kinova Web Application](http://gen3-7dof.lan) in a browser.

If you want to run the visaulization without connecting to the skin, run the same commands but there is no need to connect to the Arduino. Additionally, in Terminal 3 replace `vizual.launch` with `testing.launch`. In rviz instead of adding `taxel_markers`, add `random_float_array`. This can also be done by running `master_launch_testing.launch`, but if anything stops working or does not update in rviz it is advised to run the commands seperately.

# III. Files Explained

`master_launch_testing.launch`:<br>
Runs `viz.launch` (in `controller_ws`), `testing.launch` (in `wholearm_ws`), and opens an rviz window which visualizes the robot model and the `random_marker_array` topic.

`master_launch.launch`:<br>
Runs `viz.launch` (in `controller_ws`), `vizual.launch` (in `wholearm_ws`), and opens an rviz window which visualizes the robot model and the `taxel_markers` topic.

`gen_json.py:`<br>
Generates `robot.json` based on taxel configurations provided in the base_link frame. Taxels are given in base_link frame as an x, y, z in mm and a roll, pich, yaw in degrees. They are then converted to their respective link frame. In order for `robot.json` to be generated properly, taxels must be put in the respective array that corresponds to the link they are on (ie: shoulder_link, etc.) and the robot must be on and in the zero position. 

`viz_sub.py`:<br>
Reads in a `robot.json` as a dictionary, finds the transform between each link and the base link, and then uses that transform to put each taxel into the base_link frame. These new poses are represented as a marker array. Note that the marker.type is arrow, which traditionaly is orriented along the +X axis, which is corrected for in `gen_json.py`. Data is read in from the `/calibration` topic. Markers are published to `taxel_markers` topic.

`viz_sub_testing.py`:<br>
Reads in a `robot.json` as a dictionary, finds the transform between each link and the base link, and then uses that transform to put each taxel into the base_link frame. These new poses are represented as a marker array. Note that the marker.type is arrow, which traditionaly is orriented along the +X axis, which is corrected for in `gen_json.py`. Data is read in from the `random_float_array` topic. Markers are published to `random_marker_array` topic.

`viz_pub_testing.py`:<br>
A publisher which sends "fake" taxel data to `viz_sub_testing.py` as `random_float_array`.

`testing.launch`:<br>
Runs `viz_pub_testing.py` and `viz_sub_testing.py`.

`vizual.launch`:<br>
Runs `serial_node.py`, `custom_taxel_pub.py`, `inference.py`, and `viz_sub.py`.

`viz.launch`:<br>
Visualizes Yui. For more details see file. 

`serial_node.py`:<br>
Sets up serial communication inbetween the arduino and python.

`custom_taxel_pub.py`:<br>
Takes in raw capacitance data from the `/skin/taxel` topic and publishes to the `/skin/taxel_fast` topic.

`inference.py`:<br>
Reads in raw capacitance date from the `/skin/taxel_fast` topic, applies calibration model, and outputs calibrated force values to the `/calibration` topic.

`test_link_read.ino`: <br>
Goes through every one of the taxels and reads the raw capacitance values and sends it over the serial port to ROS. This results in the values being publieshd to the `/skin/taxel` topic.

`record.sh`: <br>
Records the all running topics and saves them to a bag_x.bag file, where x is the first availible integer in the folder. 

# IV.How to Properly Configure Taxels in the Visualization


This may be done in any 3D modeling CAD software, however for this iteration of the skin Fusion360 was chosen. To find a 3D-model of the Kinova Gen3 7DOF arm, go to [Kinova's website](https://www.kinovarobotics.com/resources?r=79302&s) in a browser. Upload the .STEP file into fusion.

Once the .STEP is in your workspace, create planes offset from the surface of the robot model. To do so, go into the respective link you are trying to add taxels to and hit the dropdown menu. This can be done on the left, where a component can be broken down into subcompoents. In the dropdown menu, click the orgin dropdown, and create an offset plane on the x, y, or z axis. Constructing an offset plane can be done from the top panel in the `Construct` section's dropdown by selecting `Offset Plane`.

To add in taxels, sketch points on these offset planes and then project the sketch onto the sufrace. This can be done from the top panel in the `Create` section's dropdown by selecting `Create Sketch`. Then in the new `Create` section, select `Line`. Draw a line along the center of the robot, and along the horizontal where each taxel will rest draw a line from one side of the robot to the other. Finally inbetween the centerline and the edge of the robot, draw a line down the center for each side. This has effectively created a grid to ensure that taxels will be evenly spaced. Go to `Create` again, and select `Point` from the dropdown. Add points to where each taxel should go on the grid. The grid should now be deleted, only leaving the points which represent the taxels. Again go to `Create` select `Project to Surface` and select the robot surface as your `Faces` and the points as your `Curves`.

This method works well for any taxel configuration that only is on one side of the robot, however some taxels are configured in "rings." In order to add in these taxels, a point at each side of the circle has to be added by following the same `Offset Plane` and `Project to Surface` method on each side of the robot. Then another offset plane can be created that is offset to one of these points, such that the plane runs through both points. This allows us to create a 2 point circle (`Create` > `Sketch`, `Create` > `2-Point Circle`) though both points. Finally, add a point to the circle on the sketch, and then go to (`Create` > `Circular Pattern`) and select that point as your `Objects` and the center point of the circle as your `Center Point`. A new option to specify how many objects to create will appear, as well as the amount of degrees between each object. For this skin six taxels were added to a ring, so 6 and 60 degrees were specified. 

 After adding in all the taxels onto the model's surface, get the x, y, z in mmm and roll, pitch, yaw in degrees from each of the taxels. To get orientation (roll, pitch, yaw), create an axis perpendicular to a point and at a face (`Construct` > `Axis Perpendicular to Face at Point`). Select the robot as your `Faces` and the point as your `Point`. This axis can be compared with respect to the orginal x, y, and z planes using the inspect tool in order to get the roll, pitch, and yaw. These can be added to `gen_json.py` as detailed below.

`gen_json.py` is where all the taxel poses are uploaded. These are uploaded as x, y, z, roll, pitch, yaw just as found above. Each taxel should be added to its respective link array. Once this is done and the robot is in the zero pose, run the following commands:

Terminal 1
In `wholearm_ws`:
```
cd controller_ws 
roscore
```

Terminal 2:
In `wholearm_ws`:
```
cd controller_ws
source devel/setup.bash
roslaunch yui_bathing viz.launch
```

Terminal 3:
In `wholearm_ws`:
```
cd wholearm_ws
source devel/setup.bash
rosrun wholearm_skin_ros gen_json.py
```
This will output `robot.json`.

 If rosrun fails - do the following in another terminal:
In `wholearm_ws`:
```
cd wholearm_ws
source devel/setup.bash
roscd wholearm_skin_ros/scripts
chmod +x gen_json.py
```
This allows rosrun to find gen_json.py

# V. Debugging Guide

***General Tips***

Remember to source the inner workspace (`wholearm_ws/wholearm_ws` or `wholearm_ws/controller_ws`): `source devel/setup.bash`

Make sure that master node is running: `roscore`, the robot is on, and that the arduino is connected and `test_link_read.ino` is running in the Arduino IDE.

Be patient when waiting for taxels to appear; if there are no error messages, it can take up to ~10s to see them in rviz. 

***If `master_launch_testing.launch` fails to execute:***

1. ROS is not configured to search both `controller_ws` and `wholearm_ws`: <br>

In order to use `master_launch_testing.launch`, we need to be able to run files in two ROS workspaces. Every time catkin clean and catkin build are ran in `controller_ws`, this line needs to be added to the end of the `setup.bash` file. To support this, within `controller_ws`, the `setup.bash` file must include the following line:

```
ROS_PACKAGE_PATH=/home/emprise/wholearm_ws/controller_ws:home/emprise/wholearm_ws/wholearm_ws:$ROS_PACKAGE_PATH
``` 

2. There is an error in one of the sub-launch files: <br>

Please see guide for `viz.launch` (in `controller_ws/src/yui_bathing/launch`). <br>
Please see guide for `vizual.launch` (in `wholearm_ws/src/wholearm_skin_ros/launch`).<br>
If all else fails, make sure that rviz is configured correctly.
<br>

3. There is an error with `wholearm_ws`: <br>

Make sure that the amount of taxels corresponds to the fake data or the real force data. Either there is an error within `viz_pub.py` and `inference` respectively, or the amount of taxels in `robot.json` is incorrect and the arrays representing taxels within `gen_json.py` need to be modified. <br>

***If everything is working but Yui cannot be teleoperated:***
  
1. There is an error in `controller_ws`: <br>

Go into `viz.launch` and set arg `use_admittance` to true.

***If the taxels are generated in the wrong place***

1. The robot was not in the zero position when `robot.json` was generated:

Regenerate taxels with all robot angles at 0 (this is the zero action in the kinova web application).

2. The taxel(s) is attributed to the wrong link and thus appears in the wrong place:

Go back into `gen_json.py` and change the array the taxel(s) is associated with. If it was accosiated with the correct link make sure the x, y, z were input correctly.

3. The taxel(s) is pointing in the wrong direction:

Go back into `gen_json.py` and modify the inccorect taxel(s) orientations.

***If the taxels are not updating in the visualization after you ran `gen_json.py` following Section IV**

1. All terminals were not reran properly:

The easiest way to rapidly prototype updating the taxels is through master_launch_testing.launch. This is becuase only one terminal needs to be ran again with the same command. However, if you want to update the taxels while running the visualization with real force data, you must rerun terminal 3 and 4 from Section II. If this still does not work, make sure that each terminal is closed and all of Section II is followed.

***If `viz.launch` fails to execute:***

1. The number of taxels in inference.py does not match the true amount of taxels on the robot

The number of taxels must correspond to the amount of taxels physically connected to the arduino and sucesfully sending data.

2. Make sure to pickle.load the correct file

Specify the correct file path in order for inference.py not to fail

3. The serial node may have already been launched

Make sure that the serial node is not launched in a seperate terminal. Vizual.launch will run this file and a conflict will occur.

# VI. Recording Data During a User Study

One useful feature of the whole arm skin is the ability to record data during a user study. Here is a guide on how to do this:

Right before the user study, run all of these commands minus ../record.sh in Terminal 4. This way, the enviornment is already set up, and all you have to do is run ../record.sh in Terminal 4 when you want to record a bag. 

First, connect to the Arduino and open the Arduino IDE. Then pull up `test_link.read.ino` and upload sketch.

In `wholearm_ws`:
```
cd controller_ws 
roscore
```

Terminal 2:
In `wholearm_ws`:
```
rosnode kill --all
cd controller_ws
source devel/setup.bash
roslaunch yui_bathing viz.launch
```

Terminal 3:
In `wholearm_ws`:
```
cd wholearm_ws
source devel/setup.bash
roslaunch wholearm_skin_ros vizual.launch
```

Terminal 4:
In `wholearm_ws`:
```
cd wholearm_ws
source devel/setup.bash
cd src/user_study
mkdir user_name
cd user_name
../record.sh
```

if storage is full, connect a portable storage device and run the following command:
```
cd ..
cd /media/emprise/Samsung_T5/skin_user_data/user_name
/home/emprise/wholearm_ws/wholearm_ws/src/user_study/record.sh
```
remember to specify the correct portable device name


If running record.sh fails - do the following in another terminal:
In `wholearm_ws`:

```
cd wholearm_ws
source devel/setup.bash
cd src/user_study
chmod +x record.sh
```

After the user study is over and you want to replay bag x do the following:

In `wholearm_ws`:
```
cd controller_ws 
roscore
```

Terminal 2:
In `wholearm_ws`:
```
cd controller_ws
source devel/setup.bash
rviz
```

Terminal 3:
In `wholearm_ws`:
```
cd wholearm_ws
source devel/setup.bash
cd src/user_study/user_name
rosbag play bag_x.bag
```
if storage is full, connect a portable storage device and run the following command:
```
cd ..
cd /media/emprise/Samsung_T5/skin_user_data/user_name
rosbag play bag_x.bag
```

Bag x records all of the topics to be replayed in rviz. This means the robot model and the corresponding taxel_markers topic can be added via the `add` button on the bottom left of the rviz window.

This process can also be repeated for test data. The only difference is that terminal 3 should run `testing.launch` rather than `vizual.launch`, and there is no need to connect to the Arduino.

