# Raspbot Fetch
For the final CS5510 project, wherein I program my raspbot to play fetch.
## Requirements
Due to the low processing power of the raspberry pi 3, a secondary computer is highly recommended.

The following apt packages are required:
* `ros-humble-desktop`
* `ros-dev-tools`
* `v4l-utils`
* `ros-humble-v4l2-camera`
* `ros-humble-image-transport-plugins`

Additionally, the following python packages are required:
* `smbus`
* `imutils`
* `opencv-python`

## Building
This project can be built as any other colcon project. Navigate to the project root directory (you should only see the `src` file), and run the command `colcon build`. After building, source the project by running the command `source install/local_setup.bash`.

## Running
Note: the following instructions assume use of a secondary computer.

Each command should be run in a separate terminal window.

1. Broadcast the webcam footage:
   * On the Raspbot, run the command `ros2 run v4l2_camera v4l2_camera_node --ros-args -p compression_quality:="100" -p brightness:="50"`
   * Note: specifying `compression_quality` and `brightness` isn't strictly necessary, but it's best to specify to keep results consistent.

2. Re-package footage:
   * On either the Raspbot or the secondary computer, run the command `ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=rare`

3. Run ball tracking:
   * On the secondary computer, run the command `ros2 run raspbot ball_detection`
   * This will open two windows: one showing the video output of the tracking algorithm, and another video output showing the color mask.
  
4. Run the follow script:
   * On the raspbot, run the command `ros2 run raspbot follow`
   * The raspbot will now begin tracking the ball, but will NOT begin moving.

5. Enable following:
   * Press space in the window showing the raspbot video output to allow the raspbot to follow the ball, and "fetch" it.
