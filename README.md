# Unity-OMPL


# run
```
git clone https://github.com/pac48/Unity-OMPL.git
cd Unity-OMPL/planner/
cmake -S . -B build
cd build
make install
```
Then open the project with the Unity editor. Note: you need to have ros sourced before opening. This is becuase the `LD_LIBRARY_PATH` is modified by ROS and needed for Unity to find the packages.

Once the project is opened,  pressthe play button to see path planning in realtime! 
