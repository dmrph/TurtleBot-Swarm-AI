# Redis

This document describes two Python scripts: `redis_data_upload.py` and `ultrasonic_redis_logger.py`. The first script provides a utility class for interacting with a Redis database, while the second script integrates ROS2 with Redis to process ultrasonic sensor data and log it to Redis.

## redis_data_upload.py

This script defines a `Redis_Ops` class that handles interactions with a Redis database. It is designed to store and retrieve robot and AprilTag location data.

### Class: Redis_Ops

-   **`bot_id`:** A class variable representing the bot ID.
-   **`redis_client`:** A Redis client instance connected to `localhost:6379`.
-   **`cosine_similarity(v1, v2)`:** A static method to calculate the cosine similarity between two vectors.
-   **`log_redis_data(aprilTagCoords, aprilTagOrient, botCoords, botOrient)`:** A class method to log robot and AprilTag location data to Redis.
    -   Combines AprilTag and bot coordinates and orientations into a single NumPy array.
    -   Stores the data as a byte string in Redis using the key `f"bot:{cls.bot_id}"`.
-   **`clear_redis_data()`:** A class method to clear all data in the Redis database.
-   **`retrieve_redis_data()`:** A class method to retrieve data from Redis and convert it back to a NumPy array.

### Dependencies

-   `redis`
-   `numpy`

## ultrasonic_redis_logger.py

This script defines a ROS2 node that subscribes to ultrasonic sensor data, processes it, and logs relevant information to Redis. It also controls the robot's movement based on sensor readings.

### Class: Redis_Ops

-   This class is very similar to the one from `redis_data_upload.py`, but it reads the `bot_id` from a configuration file.
-   **`__init__()`:** Initializes the Redis client and reads the `bot_id` from `config.json`.
-   **`log_redis_data(aprilTagCoords, aprilTagOrient, botCoords, botOrient)`:** Logs robot and AprilTag location data to Redis.
-   **`clear_redis_data()`:** Clears all data in the Redis database.
-   **`retrieve_redis_data()`:** Retrieves data from Redis.

### Class: UltrasonicSubscriber

-   **`__init__()`:** Initializes the ROS2 node, subscribes to the `ultrasonic` topic, and creates a publisher for the `/cmd_vel` topic.
-   **`listener_callback(msg)`:** Callback function that processes ultrasonic sensor data.
    -   Retrieves the distance from the `Range` message.
    -   Logs the distance.
    -   If the distance is less than or equal to 10, it moves the robot backward for 2 seconds and logs the robot's and AprilTag's location data to Redis.
    -   Uses dummy values for AprilTag and robot coordinates and orientations.
-   **`main(args=None)`:** Initializes ROS2, creates an `UltrasonicSubscriber` node, and spins the node.

### Dependencies

-   `rclpy`
-   `sensor_msgs.msg.Range`
-   `geometry_msgs.msg.Twist`
-   `redis`
-   `numpy`
-   `json`
-   `time`

### Configuration (config.json)

The `config.json` file should be located in the parent directory of `ultrasonic_redis_logger.py` and contain the bot ID:

```json
{
    "bot_id": 3,
    "target_ips": ["10.170.10.166", "10.170.9.14"],
    "secret_key": "488_fail",
    "apriltag_ids": [1, 2, 3, 4, 5],
    "bot_1_start_pos": [],
    "bot_2_start_pos": []
}
