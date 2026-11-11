# Vision-Guided Pick and Place Robot Arm (ROS 2 Jazzy, Gazebo Harmonic)

A simulated UR5e arm finds a coloured cube with an overhead RGB-D camera, converts the detection into a 3D position, plans a motion with MoveIt 2, and picks and places the cube. Perception can run with either a classical HSV colour detector or a YOLOv8n model, and both are evaluated.

<img width="507" height="305" alt="image" src="https://github.com/user-attachments/assets/2136b5d8-f8da-4d67-9266-3758f55e66c5" />
 
<img width="289" height="179" alt="image" src="https://github.com/user-attachments/assets/832e8a68-8ce0-4b2a-b72c-f1d2a9430301" />
<img width="615" height="381" alt="image" src="https://github.com/user-attachments/assets/92854cdd-fa06-4834-9356-707cda513085" />

## Pipeline

```
Gazebo overhead RGB-D camera
  -> ros_gz_bridge (/camera/image, /camera/depth_image, /camera/camera_info)
  -> detector: HSV colour  OR  YOLOv8n (ONNX, onnxruntime)
  -> pixel + depth + intrinsics -> 3D point (/detected/red, /detected/green)
  -> MoveIt 2 plan: hover, descend, correct, grasp, lift, carry, lower, release
  -> simulated attach/detach gripper (Gazebo DetachableJoint)
```

## Packages

| Package | Contents |
|---|---|
| `pickplace_description` | Table, cubes, decor, overhead camera (SDF) and `scene.launch.py` |
| `pickplace_perception` | `color_detector`, `pixel_to_world`, `object_detector`, `yolo_detector`, `collect_dataset`, `compare_offline` |
| `pickplace_manipulation` | `move_to_cube`, `pick_descend`, `pick_place`, `random_cube`, `run_trials` |

## Results

All trials: red cube placed at random positions (x 0.39 to 0.69 m, y 0.02 to 0.30 m) and moved to a fixed target. **Success = cube resting on the table within 30 mm of the target.** 
| Metric | Colour detector | YOLOv8n |
|---|---|---|
| Trials | 30 | 10 |
| Successful | 30 | 10 |
| Place error, median | 5.9 mm | 4.95 mm |
| Place error, mean | 7.7 mm | 6.6 mm |
| Place error, worst | 25.0 mm | 12.9 mm |
| Cycle time, median | 77 s | 84.5 s |
| Trials needing a MoveIt retry | 6 of 30 | 2 of 10 |
| Inference per frame (CPU) | 2.2 ms | 52.5 ms |

With these sample sizes the two detectors perform comparably. 30/30 and 10/10 support a true success rate of roughly above 90% and above 75% respectively, not exactly 100%.

<img width="535" height="251" alt="trails_table2" src="https://github.com/user-attachments/assets/fbed5e5d-eaab-48d4-84cf-10dc7f6a81b4" /> 
<img width="596" height="337" alt="image" src="https://github.com/user-attachments/assets/39044f3f-d8f9-49fb-b3b9-620f2797e033" />





### YOLOv8n training

- Dataset: 153 images (124 train, 29 val), both cubes at random positions and rotations, captured from the simulated camera.
- Labels were generated automatically from the colour detector's bounding boxes, so the model learns to reproduce it.
- Trained 50 epochs, `yolov8n`, 640 px, on an RTX 2050.
- Validation: precision 0.999, recall 1.000, mAP50 0.995, mAP50-95 0.977.
- Offline on the 29 val images: both detectors found both cubes in 29/29; mean centre difference 0.69 px (max 1.19 px).
- Live check at 3 random positions against the true cube position: error 0.2 to 6.4 mm.
- Exported to ONNX and run with onnxruntime, because the system OpenCV (4.6) cannot parse the YOLOv8 ONNX graph.

<img width="1920" height="1464" alt="image" src="https://github.com/user-attachments/assets/a69f6fda-c283-4d26-9709-19567b7de4f5" />

<img width="540" height="255" alt="image" src="https://github.com/user-attachments/assets/ee6e8180-24cf-4207-b716-9232cea7dd18" />


## Limitations

- **The gripper is simulated.** Grasping uses Gazebo's `DetachableJoint` plugin to weld and release the cube. There is no finger contact, friction or grip force.
- **Placement correction uses ground truth.** After carrying the cube, the node reads its true pose from Gazebo and corrects the arm position. A real robot would need a camera or other sensor for this.
- **Only the red cube is picked.** The gripper plugin is bound to `red_cube`.
- **YOLO is evaluated on one scene.** All images share the same lighting and background, and the labels come from the colour detector, so the high scores show agreement with that detector, not general robustness.
- Tuning runs were done before the final trials, so the trial numbers reflect the final configuration (tool tilt tolerance 0.05 rad, 30 s planning time, release verification). Two early runs with a stuck cube happened before the release check was added, and their cause is unexplained.
- The simulation renders on the CPU (WSL), so the camera is slow and the first MoveIt plan can take 30 to 40 s.

## Setup

Tested on Ubuntu 24.04 (WSL2), ROS 2 Jazzy, Gazebo Harmonic.

```bash
sudo apt install ros-jazzy-ur-simulation-gz ros-jazzy-ur-moveit-config ros-jazzy-moveit \
  ros-jazzy-ros-gz ros-jazzy-ros2-control ros-jazzy-ros2-controllers python3-opencv
pip3 install --user --break-system-packages onnxruntime

mkdir -p ~/pickplace_ws && cd ~/pickplace_ws
git clone <this repo> src
colcon build
source install/setup.bash
```

## Run

```bash
# Terminal 1: simulation (wait about 40 s)
ros2 launch pickplace_description scene.launch.py

# Terminal 2: one detector (never both)
ros2 run pickplace_perception object_detector      # colour
# or
ros2 run pickplace_perception yolo_detector        # YOLOv8n, needs models/best.onnx

# Terminal 3: one pick and place
ros2 run pickplace_manipulation pick_place red

# Evaluation: N trials with start seed S, results in trials.csv
ros2 run pickplace_manipulation run_trials 5 1000

```


### Train YOLO (separate environment, simulation closed)

```bash
ros2 run pickplace_perception collect_dataset 50 2     # repeat with new seeds
python3 -m venv --system-site-packages ~/yolo_env && source ~/yolo_env/bin/activate
pip install torch torchvision ultralytics
yolo detect train data=dataset/data.yaml model=yolov8n.pt imgsz=640 batch=8 epochs=50
yolo export model=<path>/best.pt format=onnx imgsz=640 opset=12
```

Do not source `yolo_env` when running ROS nodes (it uses numpy 2, ROS uses numpy 1).

## Notes

- Launch quirks are documented in `NOTES.md`: the UR launch accepts no world file, so models are spawned at runtime; Gazebo's Sensors system is loaded with `gz service`; `ros_gz_sim create` ignores the SDF `<pose>`, so poses are passed as arguments.
- The overhead camera cannot see a cube while the arm is above it, so the position is detected once before each pick.
