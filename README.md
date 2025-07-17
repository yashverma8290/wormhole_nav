# Multi Map Navigation using Wormholes (ROS 2 Jazzy)

This is my project for ANSCER Robotics Assignment where I made a robot which can go from one map to another using wormhole logic and custom ROS 2 action.

---

# What is this project?

This robot is made using:
- ROS 2 Jazzy
- Nav2 (for navigation)
- AMCL (for localization)
- SQLite3 (for saving wormhole data)
- Custom action server (to send goal in multi-map environment)
- URDF robot (dummy robot, just simple blue box)
- TF (Transform tree to connect all frames)
- RViz (to show robot and maps visually)

---

# Package Name

The package name is:

worhole_nav


---

# Folder Structure (Important Files)

worhole_nav/
│
├── launch/
│   └── multi_nav_bringup.launch.py    ← full system launch
│
├── config/
│   ├── nav2_params.yaml               ← AMCL and Nav2 config
│   └── nav2_default_view.rviz         ← RViz display setup
│
├── urdf/
│   └── dummy_bot.urdf                 ← robot model (blue box)
│
├── action/
│   └── MultiMapNav.action             ← custom action definition
│
├── src/
│   └── wormhole_navigator.cpp        ← main logic here
│
├── switch_map.sh                     ← bash script to switch maps
├── CMakeLists.txt
└── package.xml

---

# How to Run the Project

Use these steps from a fresh terminal:

1) Build the project

cd ~/multi_map_ws
colcon build --packages-select worhole_nav
source install/setup.bash

2) Launch everything

ros2 launch worhole_nav multi_nav_bringup.launch.py

(yeh command pura system chala deta hai — nav2, rviz, tf, sab kuch)

---

# How to Send Goal to Robot

After launch, in a new terminal, run:

source ~/multi_map_ws/install/setup.bash

ros2 action send_goal /navigate_multimap worhole_nav/action/MultiMapNav "{x: 1.0, y: 1.0, map_name: 'room1'}"

It will print:

Goal accepted with ID: ...

---

# How to See Output in RViz

- After launch, RViz opens automatically
- If you see only 3D axes, add displays manually:
  - RobotModel → Set Fixed Frame to map
  - TF → To see the transform tree
  - Map → Topic: /map
  - LaserScan → Topic: /scan (optional)

# Then go to File → Save Config As and overwrite nav2_default_view.rviz inside config/ folder.

(This step is needed to save the structure of the rviz)

---

# Wormhole Working Concept

If you send goal in a different map, it will:

1. Look in the wormholes.db (SQLite file) for a path from current map to target map
2. Go to wormhole coordinates in current map
3. Call switch_map.sh with new map YAML
4. Then go to the final destination

(Basically teleport nahi karta, smart navigation karta hai)

---

# Debugging Tips

- To check if URDF is loaded:

ros2 topic echo /robot_description

- To see transform tree:

ros2 run tf2_tools view_frames
xdg-open frames.pdf

- To check if action is working:

ros2 action list
ros2 action info /navigate_multimap

---

# Final Note

I have done all the tasks as written in the assignment:
- Created custom action
- Loaded URDF and TF properly
- Implemented wormhole logic with database
- Used Nav2 and AMCL with correct parameters
- Showed robot in RViz with correct config
- Sent test goals and verified working

---
Made by:
Yash Verma

(Assignment for ANSCER Robotics Internship Test)