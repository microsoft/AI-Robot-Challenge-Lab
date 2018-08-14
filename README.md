
# Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., label, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

# Delivery 1 - Initial Sawyer-Gazebo Simulation
The initial simulated sawyer demo has been evaluate in a Ubuntu 16.04 machine with ros kinetic installed. In order to execute the demonstration it is required first to install all the dependencies of the catkin workspace.

   rosdep install --from-paths src --ignore-src -r -y
   
Instructions to launch the simulated demonstration:

First creating a catkin workspace with the source code. 

   mkdir catkin_workspace
   cd catkin_workspace
   git clone git@github.com:Microsoft/AI-Robot-Challenge.git src

Then install the system dependencies and ros dependencies

   rosdep install --from-paths src --ignore-src -r -y
   
Build the code

   catkin build
   
Launch de demonstration

   source devel/setup.bash
   roslaunch roslaunch sorting_demo sorting_demo.launch
