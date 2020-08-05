# ROS_HomeServiceRobot
The HomeServiceRobot project aims at programming a home service robot to pick up and drop off object autonumously in an environment without knowing its initial pose. The SLAM (Simultaneous Localization And Mapping)
algorithm is implememnted to keep track of the robot current pose by MCL (Monte Carlo Localization) with known map assumption and the current map in robot's head by occupancy grid mapping with known robot pose assumption, which is essentially a close feedback loop process. 
ROS Navigation stack will be used, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan our robot trajectory from pick-up to drop-off goal position.

## Installation
1. Open a new terminal and build the catkin workspace:

    * `$ mkdir -p /home/workspace/catkin_ws/src`
 
    * `$ cd /home/workspace/catkin_ws/src`
 
    * `$ catkin_init_workspace`
 

2. Make and compiler the project (**remember to source the environment variables in every new opened terminals**):
 
    * `cd /home/workspace/catkin_ws`
 
    * `catkin_make`
 
    * `source devel/setup.bash`

3. Install the `xterminal` in workspace:
    
    * `sudo apt-get install xterm`
    
4. Clone the project under  **/home/workspace/catkin_ws/src** directory:

    * `git clone https://github.com/tonyli0130/ROS_HomeServiceRobot.git`
 
 
5. Update and upgrade the workspace image to get the lattest features of Gazebo, open a terminal and write the following statements:

    * `$ sudo apt-get update && sudo apt-get upgrade -y`

## Usage

1. Now run the HomeServiceRobot project, first run the `test_slam.sh` shell script to test if the SLAM works or not in RViz:
    
    **(Add executable permission to each shell scripts by `chmod +x /SHELL_SCRIPT_NAME` before running them)**

    * `./test_slam.sh`
    
    You should be able to control the robot in the xterminal with `teleop` node published and the map should generate simultaneously in RViz, it is not necessary to generate the entire map:
    
    
    ![test_slam](https://user-images.githubusercontent.com/60047845/89350604-f4838880-d675-11ea-820d-cd79596d56b9.PNG)
    
    
    
    
    
2. Next run the `test_navigation` shell script to test if the navigation path planning works:

    * `./test_navigation.sh`
    
    After use the ![test_navigation_2](https://user-images.githubusercontent.com/60047845/89350639-036a3b00-d676-11ea-976d-47a8eb9c48be.PNG) button to locate goal position, the robot should move toward the goal autonomously as shwon below:
    
    
    ![test_navigation_1](https://user-images.githubusercontent.com/60047845/89350628-fc432d00-d675-11ea-9ab9-bd152420b66c.PNG)
    



3. Next run the `pick_onjects` shell script to test if the robot is able to move toward the pick-up goal and drop-off goal location autonumously:

    * `./pick_objects.sh`
    
    
    ![pick_objects](https://user-images.githubusercontent.com/60047845/89350659-0a914900-d676-11ea-9c75-35da625561d9.PNG)
    
   
    The pick-up and drop-off goal are requested to the action server, more details located in `pick_objects.cpp` file.
  
  
  
 
4.  Next run the `add_markers` shell script to test if the marker which represetns the virtual object apppears in the RViz:

      * `./add_markers.sh`
      
      The marker is a red cube as shown below (top view):
      
      ![add_markers](https://user-images.githubusercontent.com/60047845/89350674-111fc080-d676-11ea-89fb-1c20d9f16ec3.PNG)
      
  
  
  
5.  After running all the testing shell scripts and no error shows up, then it is time to run the main `home_service.sh` shell script to launch the entire project:

      * `./home_service.sh`
      
      The robot moves toward the pick-up goal to pick up the cube object. After arriving the pick-up goal, the robot rests for 5 seconds and heads up to drop-off goal:
      
      
      ![home_service_1](https://user-images.githubusercontent.com/60047845/89350687-1b41bf00-d676-11ea-93be-0613c328af09.PNG)
      
      
      ![home_service_2](https://user-images.githubusercontent.com/60047845/89350705-23016380-d676-11ea-9159-4869dcb68317.PNG)
      
      
      Finally, the robot should drop off the cube object at dropp-off goal sucessfully:
      
      
      ![1](https://user-images.githubusercontent.com/60047845/89353778-c8b7d100-d67c-11ea-95dc-56daf72a9690.PNG)

      
      
