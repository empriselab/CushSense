# I. Directory Diagram
Below is a tree reperesentation of the repository directory. It includes necessary files that are relevant to the skin visualization; these files will be detailed further in Section III.
```
arduino
    ├── skin.ino
    ├── skin_fast.ino
    ├── TaxelData.h
wholearm_skin_ros
    ├── launch
        ├── vizual.launch
    ├── msg
        ├── TaxelData.msg
    ├── scripts
        ├── custom_taxel_pub.py
        ├── inference.py
        ├── viz_sub.py
        ├── data_collection.py
        ├── calibration.py
        ├── digitalfilter.py
    ├── ...
```
# II. Running the Visualization
First connect to the Arduino and open the Arduino IDE. Then pull up `skin.ino` and upload sketch.
Then, to run the visualization run the following commands:

Terminal 1:
```
roscore
```
Terminal 2:
```
source devel/setup.bash
roslaunch wholearm_skin_ros vizual.launch
```
Terminal 3:
```
source devel/setup.bash
rviz
```
The result of these commands will be an rviz window. Within this window, click `add` at the bottom left, and under the category `By Topic` click `taxel_markers`. You may also need to import the robot model you are using if you want to visualize the skin attaching to a moving robot arm.
# III. Files Explained
`skin.ino`:<br>
Reads digital data from all taxels and publish it to a ros node.

`fast_skin.ino`:<br>
A latest change made to make the data-reading process much faster.

`serial_node.py`:<br>
This is a file mentioned in the lauch file but not included in this repository. To get this file, you only need to install [rosserial package](https://wiki.ros.org/rosserial). It can set up serial communication between the arduino and python.

`custom_taxel_pub.py`:<br>
Takes in raw digital data from the `/skin/taxel` topic and publishes to the `/skin/taxel_fast` topic.

`inference.py`:<br>
Reads in raw digital date from the `/skin/taxel_fast` topic, applies calibration model, and outputs calibrated force values to the `/calibration` topic.

`gen_json.py:`<br>
You can write this file according to the robot arm you use. This file is supposed to generate `robot.json` based on taxel configurations provided in the base_link frame. Taxels are given in base_link frame as an x, y, z in mm and a roll, pich, yaw in degrees. They are then converted to their respective link frame. In order for `robot.json` to be generated properly, taxels must be put in the respective array that corresponds to the link they are on (ie: shoulder_link, etc.) and the robot must be on and in the zero position. 

`viz_sub.py`:<br>
Reads in a `robot.json` as a dictionary, finds the transform between each link and the base link, and then uses that transform to put each taxel into the base_link frame. These new poses are represented as a marker array. Note that the marker.type is arrow, which traditionaly is orriented along the +X axis, which is corrected for in `gen_json.py`. Data is read in from the `/calibration` topic. Markers are published to `taxel_markers` topic.

# IV. How to Properly Configure Taxels in the Visualization

This may be done in any 3D modeling CAD software, however for this iteration of the skin Fusion360 was chosen. To find a 3D-model of the Kinova Gen3 7DOF arm, go to [Kinova's website](https://www.kinovarobotics.com/resources?r=79302&s) in a browser. Upload the .STEP file into fusion.

Once the .STEP is in your workspace, create planes offset from the surface of the robot model. To do so, go into the respective link you are trying to add taxels to and hit the dropdown menu. This can be done on the left, where a component can be broken down into subcompoents. In the dropdown menu, click the orgin dropdown, and create an offset plane on the x, y, or z axis. Constructing an offset plane can be done from the top panel in the `Construct` section's dropdown by selecting `Offset Plane`.

To add in taxels, sketch points on these offset planes and then project the sketch onto the sufrace. This can be done from the top panel in the `Create` section's dropdown by selecting `Create Sketch`. Then in the new `Create` section, select `Line`. Draw a line along the center of the robot, and along the horizontal where each taxel will rest draw a line from one side of the robot to the other. Finally inbetween the centerline and the edge of the robot, draw a line down the center for each side. This has effectively created a grid to ensure that taxels will be evenly spaced. Go to `Create` again, and select `Point` from the dropdown. Add points to where each taxel should go on the grid. The grid should now be deleted, only leaving the points which represent the taxels. Again go to `Create` select `Project to Surface` and select the robot surface as your `Faces` and the points as your `Curves`.

This method works well for any taxel configuration that only is on one side of the robot, however some taxels are configured in "rings." In order to add in these taxels, a point at each side of the circle has to be added by following the same `Offset Plane` and `Project to Surface` method on each side of the robot. Then another offset plane can be created that is offset to one of these points, such that the plane runs through both points. This allows us to create a 2 point circle (`Create` > `Sketch`, `Create` > `2-Point Circle`) though both points. Finally, add a point to the circle on the sketch, and then go to (`Create` > `Circular Pattern`) and select that point as your `Objects` and the center point of the circle as your `Center Point`. A new option to specify how many objects to create will appear, as well as the amount of degrees between each object. For this skin six taxels were added to a ring, so 6 and 60 degrees were specified. 

 After adding in all the taxels onto the model's surface, get the x, y, z in mmm and roll, pitch, yaw in degrees from each of the taxels. To get orientation (roll, pitch, yaw), create an axis perpendicular to a point and at a face (`Construct` > `Axis Perpendicular to Face at Point`). Select the robot as your `Faces` and the point as your `Point`. This axis can be compared with respect to the orginal x, y, and z planes using the inspect tool in order to get the roll, pitch, and yaw. These can be added to `gen_json.py` as detailed below.

`gen_json.py` is where all the taxel poses are uploaded. These are uploaded as x, y, z, roll, pitch, yaw just as found above. Each taxel should be added to its respective link array. Once this is done and the robot is in the zero pose, run the following commands:

Terminal 1
```
roscore
```
Terminal 2:
```
source devel/setup.bash
rosrun wholearm_skin_ros gen_json.py
```
This will output `robot.json`.

If rosrun fails - do the following in another terminal:
```
cd wholearm_ws
source devel/setup.bash
roscd wholearm_skin_ros/scripts
chmod +x gen_json.py
```
This allows rosrun to find gen_json.py

# V. Steps to perform calibration
Due to the differences in physical structure between taxels, we decided to do a calibration process for each taxel. Steps are as followed.

Plug in the Arduino. Make sure that the connector to the multiplexer is connected to the "top" of the multiplexer (words are upright).

The arduino code is in `skin.ino`. It publishes an array with all of the digital data from each taxel. To give Arduino connection permission, use 'sudo chmod a+rw /dev/ttyACM0' (change this to your own port).

Terminal 1:
```
roscore
```
Terminal 2:
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```
note: change this to your own port and baud rate. 

Terminal 3:
```
rosrun wholearm_skin_ros custom_taxel_pub.py
```
Terminal 4:
```
roslaunch tams_wireless_ft wireless_ft.launch
```
Terminal 5:
```
rosrun wholearm_skin_ros data_collection.py
```
After these, it will start printing values once the zero offset for taxel values has been calculated. Post this, you can start collecting data using the intruder.

Once you're done, press `Ctrl + c` to stop the script and save the data.

After this, run `rosrun wholearm_skin_ros calibration.py` to fit the calibration function. Make sure to change the file name in the script to the file name of the data you just collected. 

To test the function, run `rosrun wholearm_skin_ros inference.py`. This will publish the force readings obtained from the skin on `/calibration` topic.

You can use `rqt_plot` to view the plots for `/calibration` and ground truth data on `forque/forqueSensor/wrench/force/z`.
