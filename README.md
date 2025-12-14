🌿 Webots Drone Pollution Detection System
📌 Overview

This project simulates an autonomous drone that scans a forest environment to detect pollution sources and reports their GPS coordinates in real time. Pollution is represented by red-colored objects, and detection is performed using an onboard camera and GPS sensor within the Webots simulation environment.

📂 Project Structure
pollution_detection_drone/
├── worlds/
│   └── forest_pollution.wbt        # Main simulation world
├── controllers/
│   └── drone_controller/
│       └── drone_controller.py     # Autonomous drone control logic
└── README.md                       # Project documentation

▶️ How to Run the Simulation

Open Webots

Click File → Open World

Select worlds/forest_pollution.wbt

Click the Play (▶) button

Observe the console output for pollution detection alerts and GPS coordinates

🚁 System Functionality

Drone takes off to a fixed altitude of 10 meters

Navigates the forest using a grid / waypoint-based scanning pattern

Onboard camera detects red-colored pollution sources

When pollution is detected:

GPS coordinates are captured

Alert is printed in the console

After completing the scan, the drone returns to its starting position

⚙️ Customization Options

You can modify the system behavior by editing:

controllers/drone_controller/drone_controller.py

Configurable Parameters
Parameter	Description	Default Value
TARGET_ALTITUDE	Drone scanning altitude	10 meters
RED_THRESHOLD	Color detection sensitivity	0.2
SCAN_AREA_SIZE	Total scan area	30 × 30 meters
SCAN_STEP	Distance between scan lines	5 meters
🚨 Pollution Detection Output

When a pollution source is detected, the console displays:

==================================================
[ALERT] POLLUTION DETECTED!
==================================================
Location coordinates:
  X: -5.23 meters
  Y: -10.15 meters
  Z: 10.00 meters (altitude)
Time: 45.2 seconds
==================================================

🛠 Troubleshooting

Problem: Drone does not move
Solution: Ensure the Play (▶) button is pressed in Webots

Problem: No pollution detected
Solution:

Reduce RED_THRESHOLD value

Verify red pollution objects exist in the world file

Problem: Controller errors
Solution:

Check Python installation

Ensure controller folder name matches controller file name

✅ Requirements

Webots R2023b or later

Python 3.8 or higher

📖 References

Webots Documentation:
https://cyberbotics.com/doc/guide/index
