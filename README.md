# WEBOTS DRONE POLLUTION DETECTION SYSTEM

## OVERVIEW

This project simulates a drone that autonomously scans a forest region to detect pollution sources and reports their GPS coordinates.

---

## PROJECT STRUCTURE
```
pollution_detection_drone/
├── worlds/
│   └── forest_pollution.wbt       # Main simulation world
├── controllers/
│   └── drone_controller/
│       └── drone_controller.py    # Drone control logic
└── README.md                      # This file
```

---

## HOW TO RUN

1. Open Webots
2. Go to **File → Open World**
3. Select: `worlds/forest_pollution.wbt`
4. Click the **Play button (▶)**
5. Watch the console for pollution detection alerts

---

## WHAT IT DOES

- Drone takes off to 10 meters altitude
- Flies in a grid pattern over the forest
- Camera detects red pollution sources
- Prints GPS coordinates when pollution is found
- Returns to start position when scan is complete

---

## CUSTOMIZATION

Edit `controllers/drone_controller/drone_controller.py`:

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `TARGET_ALTITUDE` | Change scanning height | 10 meters |
| `RED_THRESHOLD` | Adjust detection sensitivity | 0.2 |
| `SCAN_AREA_SIZE` | Change area coverage | 30x30 meters |
| `SCAN_STEP` | Distance between scan lines | 5 meters |

---

## POLLUTION DETECTION

When pollution is detected, you'll see output like:
```
==================================================
[ALERT] POLLUTION DETECTED!
==================================================
Location coordinates:
  X: -5.23 meters
  Y: -10.15 meters
  Z: 10.00 meters (altitude)
Time: 45.2 seconds
==================================================
```

---

## TROUBLESHOOTING

| Problem | Solution |
|---------|----------|
| Drone doesn't move | Make sure you pressed Play button |
| No pollution detected | Lower `RED_THRESHOLD` value or verify pollution sources exist |
| Controller errors | Check Python installation and controller name matches folder |

---

## REQUIREMENTS

- **Webots R2023b or later**
- **Python 3.8+**

---

## RESOURCES

For help, visit: [Webots Documentation](https://cyberbotics.com/doc/guide/index)

---

## LICENSE

This project is provided as-is for educational purposes.
