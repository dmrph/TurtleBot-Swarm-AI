# Ultrasonic Node
## Setup
1. Copy the `ultrasonic` folder in `Ultrasonic/ultrasonic` to your `~/shared_students/turtlebot3_ws/src` directory.
```
cp -r Ultrasonic/ultrasonic ~/shared_students/turtlebot3_ws/src/ultrasonic
```

2. Add the following lines to `~/shared_students/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py` right after the `turtlebot3_node` node in the same script:
```
Node(
    package='ultrasonic',
    executable='ultrasonic_publisher',
    output='screen',
),
```

3. Navigate to `~/shared_students/turtlebot3_ws` and run the following command to compile the ultrasonic package:
```
cd ~/shared_students/turtlebot3_ws
colcon build --packages-select ultrasonic
```

4. Navigate to the `~/shared_students/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup` folder and run the following command to compile the script changes:
```
cd ~/shared_students/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup
colcon build
```

5. Make sure `setup.bash` is sourced by running the command:
```
source ~/shared_students/turtlebot3_ws/install/setup.bash
```

6. Run the `robot.launch.py` script and leave it running.
```
ros2 launch turtlebot3_bringup robot.launch.py
```

7. Verify that the ultrasonic topic has been created.
```
ros2 topic list
```

8. Verify that valid ultrasonic measurements are logged to console.
```
ros2 run ultrasonic ultrasonic_subscriber
```

9. Replace your `pythonAiAgents` folder with our `pythonAiAgents` folder, this includes a proper SonicAgent class that subscribes to the ultrasonic topic and signals when objects come to close. This also includes multithreading for concurrent ros2 nodes to avoid blocking.