# Mobile robot control using ArUco markers

In this repository you'll find the proyect's article and slideshows (In spanish), Python scripts and Arduino sketches developed during my stay project on the Polytechnic University of Atlacomulco.

With this, you can control a large number of robots using AruCo markers on top of them to identify them with a camera.

The main script **robotFormation.py** creates a window in which you can select a robot with a left click and tell it where to go in the window with a right click.
Before you can use this script, you need to create a file with the perspective transformation matrix
using the script **charucoPerspectiveCalibrate.py**, this requires a ChArUco board at the height the markers on the robot would be.