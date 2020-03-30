# ros-quadricopter-controller
This project was done in partial fulfillment of the requirement for the
award of the degree of Bachelor of Technology.

The simulation of the flight of a drone and its autonomous avoidance of obstacles is done. This is achieved by applying
a two-neuron recurrent neural network. 

The programming of the flight of the drone is done in lua inside V-REP while the neural control 
handling obstacle avoidance is programmed in Python 2.7 both of which are interfaces using ROS nodes and messages.

### Software requirements
1. Ubuntu 18.04 (required for ROS Melodic)
2. V-REP PRO EDU (for the simulation environment)
3. ROS melodic (for communication between the V-REP and the neural controller)

### Setting up the development environment
1. Once you're done installing the softwares listed above, setup ROS using the instructions given [here](https://wiki.ros.org/melodic/Installation/Ubuntu).
2. Create a catkin workspace in home or any other directory using the following commands:
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    ````
3. Add the newly created catkin_ws into your .bashrc or .zshrc file using:
    ```
    echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.bashrc
    ```
4. Clone this repository into the src folder of your catkin_ws
    ```
    cd ~/catkin_ws/src && git clone https://github.com/mukul29/ros-quadricopter-controller
    ```

### Running the project
1. Start roscore by running the command (this has to be done before launching V-REP).
    ```
    roscore
    ```
2. Launch V-REP and load the scene Quadricopter.ttt present inside ros-quadricopter-controller/vrep_scenes.
3. Start the simulation in V-REP.
4. Run the script responsible for autonomous obstacle avoidance in another terminal by issuing the following command:
    ```
    rosrun ros-quadricopter-controller quadricopterController.py
    ```
5. Optionally, run the mapViewer.py script which marks the positions where an obstacle is encountered using:
    ```
    rosrun ros-quadricopter-controller mapViewer.py
    ```
### References
1. C. K. Pedersen and P. Manoonpong, “Neural Control and Synaptic Plasticity for Adaptive Obstacle Avoidance of Autonomous Drones,” *Lecture Notes in Artificial Intelligence*, 2018.
2. Devos, Arne, Emad Ebeid, and Poramate Manoonpong. "Development of Autonomous Drones for Adaptive Obstacle Avoidance in Real World Environments." *2018 21st Euromicro Conference on Digital System Design (DSD)*.
