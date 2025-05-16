# Mongo

## check_latest_entries.py

This script retrieves and displays the latest health data entries for a specified robot from a MongoDB database (`turtlebot_db`, `bot_health`). It formats timestamps to local time, calculates the time difference from the current time, and presents a summarized view of the robot's recent health status, including database connectivity, error counts, battery level, pose, ROS topic statuses, and the last access time of the entry. It also updates the `last_accessed` field for each retrieved entry.

*Note:* When run directly, this script also activates a function (`continuously_log_pose`) that indefinitely logs mock pose data to the database every 3 seconds, which is separate from its primary display function.

## Dependencies:

*   `sys`
*   `json`
*   `time`
*   `pymongo`
*   `datetime`

## Methods/Functions:

*   **get_latest_entries(bot_id, limit):** Connects to MongoDB, retrieves the specified number (`limit`) of the most recent health entries for the given `bot_id`, formats key information (timestamp, statuses, data points), and returns it as a list of dictionaries.
*   **get_robot_pose():** Generates static mock pose data (used by `continuously_log_pose`).
*   **continuously_log_pose():** Runs in an infinite loop, inserting mock pose data into the `bot_health` collection every 3 seconds. Activated when the script is run directly.

## Usage:

1.  Ensure you have Python 3 and the `pymongo` library installed.
2.  Requires a running MongoDB instance accessible at `'mongodb://10.170.9.20:27017/'` with a database named `turtlebot_db` and a collection named `bot_health`. Adjust the hardcoded connection string if needed.
3.  Run the script from the command line, optionally providing the `bot_id` and the number of entries (`limit`) to display:
    ```bash
    python check_latest_entries.py [bot_id] [limit]
    ```
    *   `[bot_id]` defaults to `"2"` if not provided.
    *   `[limit]` defaults to `5` if not provided.
4.  The script will print the formatted recent entries and then start logging mock pose data continuously until interrupted (Ctrl+C).

---

## config.json

This file serves as a JSON configuration store for the `logger.py` ROS2 node (and potentially other related scripts). It defines various parameters required for the robot logger's operation.

## Structure/Content:

*   **bot**: Contains settings specific to the robot.
    *   `id` (str): The unique identifier for the robot instance using this configuration.
*   **ros2**: Contains ROS2 specific configurations.
    *   `node_name_prefix` (str): A prefix used for naming the ROS2 node.
    *   `topics` (dict): A mapping of logical topic names (e.g., "odometry") to their actual ROS2 topic strings (e.g., "/odom").
*   **mongodb**: Contains MongoDB connection and collection details.
    *   `uri` (str): The MongoDB connection string.
    *   `database` (str): The name of the MongoDB database to use.
    *   `sensor_collection` (str): The name of the collection for storing sensor data logs.
    *   `health_collection` (str): The name of the collection for storing health status logs.
    *   `mission_collection` (str): The name of the collection for storing mission-related data.
*   **logging**: Contains settings related to logging frequency and data retention.
    *   `sensor_interval_seconds` (float): How often (in seconds) to log sensor data.
    *   `health_interval_seconds` (float): How often (in seconds) to perform health checks and log health data.
    *   `mission_interval_seconds` (float): How often (in seconds) to log mission status updates.
    *   `retention_hours` (int): How long (in hours) to keep log entries before cleanup.
*   **sensors**: Contains flags and settings for enabling/disabling specific sensors.
    *   `camera`, `lidar`, `ultrasonic`, `battery` (dict): Each contains an `enabled` (bool) flag and potentially other sensor-specific settings (e.g., `sample_rate` for lidar).

## Usage:

1.  This file should be placed where the `logger.py` script (or other consuming scripts) can access it (e.g., in the same directory or a path specified via command-line argument).
2.  Edit the values within this file to match your specific robot ID, ROS2 topic names, MongoDB setup, desired logging rates, and enabled sensors.

---

## logger.py

This script implements a ROS2 node (`BotLogger`) for TurtleBot robots, designed for logging data during autonomous missions (like charm collection). It subscribes to various sensor topics (Odometry, LiDAR, Ultrasonic, Battery State), collects system health information (network status, ROS topic liveliness, errors), and logs this data periodically to specified MongoDB collections. It also includes functionality for cleaning up old log entries based on a configured retention period.

## Dependencies:

*   `rclpy`
*   `pymongo`
*   `bson`
*   `datetime`
*   `json`
*   `math`
*   `socket`
*   `psutil`
*   `subprocess`
*   `threading`
*   `argparse`
*   `os`
*   `nav_msgs.msg` (Odometry)
*   `sensor_msgs.msg` (LaserScan, BatteryState, Range)
*   `std_msgs.msg` (String, Bool, Float32 - *Note: Imported but not directly used in provided callbacks*)

## Classes:

*   **BotLogger(Node):**
    *   **Attributes:**
        *   `config` (dict): Configuration loaded from `config.json`.
        *   `bot_id` (str): Unique identifier for this robot instance.
        *   `mongo_connected` (bool): Flag indicating the current MongoDB connection status.
        *   `client` (MongoClient): The PyMongo client instance.
        *   `db` (Database): The PyMongo database instance.
        *   `sensor_collection` (Collection): MongoDB collection for sensor logs.
        *   `health_collection` (Collection): MongoDB collection for health logs.
        *   `mission_collection` (Collection): MongoDB collection for mission data logs.
        *   `data_lock` (Lock): A threading lock to ensure thread-safe access to shared data.
        *   `location` (dict): Stores the latest robot position (`x`, `y`) and `orientation` (yaw in degrees).
        *   `lidar_data` (dict): Stores processed data from the last received LaserScan message.
        *   `ultrasonic_data` (dict): Stores the latest distance, timestamp, and status from the Range sensor.
        *   `health_data` (dict): Stores current health status including battery, network info, errors, and ROS topic/sensor statuses.
        *   `mission_data` (dict): Stores mission-specific data (e.g., charms collected/detected - currently basic structure).
        *   `sensor_log_timer` (Timer): ROS2 timer triggering `log_sensor_data`.
        *   `health_check_timer` (Timer): ROS2 timer triggering `perform_health_check`.
        *   `mission_update_timer` (Timer): ROS2 timer triggering `update_mission_status`.
        *   `maintenance_timer` (Timer): ROS2 timer triggering `cleanup_old_logs`.
    *   **Methods:**
        *   `__init__(config_path)`: Initializes the node, loads config, connects to MongoDB, sets up subscribers and timers.
        *   `load_config(config_path)`: Loads configuration from the specified JSON file, with defaults as fallback.
        *   `quaternion_to_yaw(x, y, z, w)`: Converts a ROS Quaternion message to a Yaw angle in degrees.
        *   `odom_callback(msg)`: Callback for Odometry messages; updates `location`.
        *   `lidar_callback(msg)`: Callback for LaserScan messages; updates `lidar_data` and lidar health status.
        *   `ultrasonic_callback(msg)`: Callback for Range messages; updates `ultrasonic_data` and ultrasonic health status.
        *   `battery_callback(msg)`: Callback for BatteryState messages; updates battery info in `health_data`.
        *   `update_topic_status(topic_name)`: Marks a given ROS topic as 'ONLINE' in `health_data`.
        *   `perform_health_check()`: Periodically checks topic staleness, MongoDB connection, updates network info, and triggers `log_health_data`.
        *   `update_network_info()`: Retrieves current network interface, IP address, and signal strength (if WiFi).
        *   `log_error(error_message)`: Logs an error message to the ROS logger and stores it in `health_data`.
        *   `log_sensor_data()`: Collects current sensor data (`location`, `lidar`, `ultrasonic`) and logs it to the `sensor_collection`.
        *   `log_health_data()`: Collects current health status (`battery`, `network`, `topics`, `sensors`, `errors`, `database`) and logs it to the `health_collection`.
        *   `update_mission_status()`: Collects current mission data (`location`, `charms_*`, `assigned_area`) and logs it to the `mission_collection`.
        *   `cleanup_old_logs()`: Deletes entries older than `config["logging"]["retention_hours"]` from all relevant collections.

## Methods/Functions (Standalone):

*   **main():** Parses command-line arguments (`--config`, `--bot-id`), initializes `rclpy`, creates and spins the `BotLogger` node, and handles shutdown.

## Usage:

1.  Ensure ROS2 (corresponding to your `rclpy` version) and Python 3 are installed.
2.  Install required Python libraries: `pymongo`, `psutil`.
3.  Ensure a MongoDB instance is running and accessible via the URI specified in `config.json`.
4.  Create a `config.json` file (or use the default path) and configure it with your settings (MongoDB URI, topics, bot ID, etc.).
5.  Build your ROS2 workspace if this script is part of a package.
6.  Run the node, typically using `ros2 run` or a launch file. Example using `ros2 run`:
    ```bash
    # Make sure to replace <your_package_name> with the actual package name
    ros2 run <your_package_name> logger --ros-args --params-file /path/to/your/config.json
    ```
    *   You can override the config file path using `--config` (relative to where you run) or the bot ID using `--bot-id` via standard command-line arguments before `--ros-args`.

---

## mock_mongodb_logger.py

This script provides a `TurtleBotMongoLogger` class that simulates a TurtleBot by generating random mock sensor data (location, sonar readings, LiDAR readings, and camera data as base64 string) and logging it periodically to a MongoDB database (`turtlebot_db`, `bot_logs` by default). It also includes a method to retrieve the most recent logs for testing or demonstration purposes.

## Dependencies:

*   `pymongo`
*   `datetime`
*   `json`
*   `random`
*   `base64`
*   `time`
*   `threading`

## Classes:

*   **TurtleBotMongoLogger:**
    *   **Attributes:**
        *   `bot_id` (str): The identifier for the simulated TurtleBot.
        *   `client` (MongoClient): The PyMongo client instance.
        *   `db` (Database): The PyMongo database instance.
        *   `collection` (Collection): The PyMongo collection instance used for logging.
        *   `location` (dict): Stores mock location data (`x`, `y`, `orientation`).
        *   `sonar_data` (list): Stores a rolling window of mock sonar readings.
        *   `lidar_data` (list): Stores mock LiDAR range readings.
        *   `camera_data` (str): Stores mock camera data encoded as a base64 string.
        *   `data_lock` (Lock): A threading lock for thread safety (though less critical in this mock script's main usage).
    *   **Methods:**
        *   `__init__(bot_id, mongo_uri, db_name, collection_name)`: Initializes the logger, connects to MongoDB using the provided URI, database, and collection names, and sets the `bot_id`.
        *   `generate_mock_data()`: Generates new random values for location, sonar, LiDAR, and camera data, updating the instance attributes.
        *   `log_data()`: Creates a timestamped log document containing the current mock data (`botId`, `timestamp`, `location`, `sensors`) and inserts it into the MongoDB collection.
        *   `retrieve_logs(limit)`: Queries MongoDB for the most recent log entries (up to `limit`) for the instance's `bot_id` and returns them as a list.

## Usage:

1.  Ensure you have Python 3 and the `pymongo` library installed.
2.  Ensure a MongoDB instance is running and accessible at the URI specified in the script or passed to the constructor (default: `'mongodb://10.32.34.141:27017/'`).
3.  Run the script directly from the command line:
    ```bash
    python mock_mongodb_logger.py
    ```
4.  When run directly, the script initializes a logger for `bot_id = "2"`, generates and logs mock data 5 times (with a 1-second pause between logs), and then retrieves and prints the 3 most recent logs in JSON format.

---

## test_connection.py

This script is designed to test the connection to a specific MongoDB instance and verify basic database operations (insert, find, delete) relevant to the TurtleBot logging schema. It connects to a hardcoded MongoDB URI, database, and collection, performs the test operations with a sample document, and reports success or failure at each step.

## Dependencies:

*   `pymongo`
*   `datetime`
*   `json`
*   `base64`
*   `sys`

## Methods/Functions:

*   **test_mongodb_connection():**
    1.  Attempts to connect to the MongoDB instance defined by `MONGO_URI`.
    2.  Verifies the connection using `server_info()`.
    3.  Accesses the database (`DB_NAME`) and collection (`COLLECTION_NAME`).
    4.  Creates a sample log document (`test_doc`) mimicking the logging schema.
    5.  Inserts the `test_doc` into the collection.
    6.  Retrieves the inserted document using its `_id`.
    7.  Deletes the `test_doc` from the collection.
    8.  Prints status messages for each step (connection, insert, retrieval, deletion).
    9.  Returns `True` if all steps succeed, `False` if any exception occurs.
    10. Ensures the MongoDB client connection is closed in a `finally` block.

## Usage:

1.  Ensure you have Python 3 and the `pymongo` library installed.
2.  Ensure a MongoDB instance is running and accessible at the hardcoded URI: `'mongodb://10.32.34.141:27017/'`.
3.  Modify the `MONGO_URI`, `DB_NAME`, and `COLLECTION_NAME` constants at the top of the script if your target database is different.
4.  Run the script from the command line:
    ```bash
    python test_connection.py
    ```
5.  The script will output the results of the connection and CRUD tests. It will exit with code `0` on success and `1` on failure.

---

## Topics.py

This script connects to a MongoDB database (`turtlebot_db`, `bot_health`) to retrieve the single most recent health entry for a specified robot ID. It then parses this entry to display the status ('ONLINE', 'OFFLINE', or 'UNKNOWN') of predefined ROS topics (`/odom`, `/ultrasonic`, `/battery_state`) based on the information stored in that log entry. The timestamp of the entry is also displayed in local time.

## Dependencies:

*   `sys`
*   `json`
*   `pymongo`
*   `datetime`

## Methods/Functions:

*   None defined (logic is executed sequentially in the script body).

## Usage:

1.  Ensure you have Python 3 and the `pymongo` library installed.
2.  Requires a running MongoDB instance accessible at `'mongodb://10.170.9.20:27017/'` with a database named `turtlebot_db` and a collection named `bot_health`, containing data logged by a compatible script (like `logger.py`).
3.  Adjust the hardcoded MongoDB connection string if necessary.
4.  Run the script from the command line, optionally providing the `bot_id`:
    ```bash
    python Topics.py [bot_id]
    ```
    *   `[bot_id]` defaults to `"2"` if not provided.
    *   *Note:* The script has an unused `limit` argument parsing, but it always processes only the single latest entry.
5.  The script will print the status of the predefined topics based on the latest available health log entry for the specified bot.

---

## TopicStatus.py

This script implements a ROS2 node (`TopicMonitor`) that actively monitors the status of several predefined ROS topics (`/odom`, `/scan`, `/ultrasonic`, `/camera/image_raw`). It subscribes to these topics and tracks the time of the last received message for each. A timer periodically checks if the time since the last message exceeds a threshold (2 seconds); if it does, the topic is marked as 'offline', otherwise it's 'online'. Another timer periodically logs the current online/offline status of all monitored topics to a MongoDB collection (`turtlebot_db`, `bot_health`).

## Dependencies:

*   `rclpy`
*   `pymongo`
*   `datetime`
*   `threading`
*   `json` (*Imported but not used*)
*   `os` (*Imported but not used*)
*   `nav_msgs.msg` (Odometry)
*   `sensor_msgs.msg` (LaserScan, Range, Image, BatteryState - *Note: BatteryState imported but no corresponding subscription/callback*)
*   `std_msgs.msg` (String - *Imported but not used*)

## Classes:

*   **TopicMonitor(Node):**
    *   **Attributes:**
        *   `client` (MongoClient): PyMongo client connected to MongoDB.
        *   `db` (Database): PyMongo database instance (`turtlebot_db`).
        *   `status_col` (Collection): PyMongo collection instance (`bot_health`) for logging status.
        *   `topic_states` (dict): Stores the current boolean status (True=online, False=offline) for monitored topics. Keys: `/odom`, `/scan`, `/ultrasonic`, `/camera/image_raw`. *Correction: Based on callbacks, battery isn't monitored despite import/initial state.*
        *   `last_msg_times` (dict): Stores the `rclpy.time.Time` of the last received message for each monitored topic.
        *   `state_lock` (Lock): Thread lock for safe access to shared state dictionaries.
        *   `logging_interval_sec` (int): Interval (in seconds) for logging status to MongoDB (hardcoded to 10).
    *   **Methods:**
        *   `__init__()`: Initializes the node, connects to MongoDB, sets up initial state, creates subscriptions (`/odom`, `/scan`, `/ultrasonic`, `/camera/image_raw`), and starts timers for state updates and logging.
        *   `odom_callback(msg)`: Updates `last_msg_times['/odom']`.
        *   `lidar_callback(msg)`: Updates `last_msg_times['/scan']`.
        *   `ultrasonic_callback(msg)`: Updates `last_msg_times['/ultrasonic']`.
        *   `camera_callback(msg)`: Updates `last_msg_times['/camera/image_raw']`.
        *   `update_topic_states()`: Timer callback; checks elapsed time for each topic in `last_msg_times` and updates `topic_states` (online/offline based on 2-second timeout).
        *   `log_status()`: Timer callback; constructs a document with current topic statuses ('online'/'offline' strings) and inserts it into the `status_col` MongoDB collection.

## Methods/Functions (Standalone):

*   **main():** Initializes `rclpy`, creates and spins the `TopicMonitor` node, and handles shutdown.

## Usage:

1.  Ensure ROS2 (corresponding to your `rclpy` version) and Python 3 are installed.
2.  Install required Python libraries: `pymongo`.
3.  Ensure a MongoDB instance is running and accessible at the hardcoded URI: `'mongodb://10.170.9.20:27017/'`.
4.  Ensure the ROS topics being monitored (`/odom`, `/scan`, `/ultrasonic`, `/camera/image_raw`) are being published elsewhere in the ROS system.
5.  Build your ROS2 workspace if this script is part of a package.
6.  Run the node, typically using `ros2 run` or a launch file:
    ```bash
    # Make sure to replace <your_package_name> with the actual package name
    ros2 run <your_package_name> TopicStatus
    ```
7.  The node will run in the background, logging topic statuses to MongoDB every 10 seconds. Check the node's log output (`INFO` level) to see the status being logged.