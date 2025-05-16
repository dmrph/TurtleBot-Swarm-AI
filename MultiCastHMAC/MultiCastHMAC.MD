# MultiCastHMAC

## Broadcast.py

This script defines a `Broadcaster` class responsible for sending UDP messages to a list of target IP addresses on a specified port. Messages contain a JSON payload (either a predefined command structure or a custom payload) and an HMAC-SHA256 signature for verification by receivers. It is designed for scenarios where a central controller needs to send authenticated commands or information to multiple robots or devices.

## Dependencies:

*   `socket`
*   `json`
*   `hmac`
*   `hashlib`
*   `base64`
*   `time`

## Classes:

*   **Broadcaster:**
    *   **Attributes:**
        *   `SECRET_KEY` (bytes): The secret key used for generating HMAC signatures. Must match the key used by receivers.
        *   `TARGET_IPS` (list): A list of strings, where each string is the IP address of a target receiver.
        *   `UDP_PORT` (int): The UDP port number to send messages to (default: 5005). Receivers must listen on this port.
        *   `BOT_ID` (str): An identifier for the bot/entity sending the broadcast (used in `broadcast_command`).
    *   **Methods:**
        *   `__init__(secret_key, target_ips, bot_id)`: Initializes the Broadcaster instance with the shared secret key, a list of target IP addresses, and the sender's bot ID.
        *   `generate_hmac(payload: dict) -> str`: Takes a Python dictionary payload, converts it to a canonical JSON string, computes the HMAC-SHA256 signature using `SECRET_KEY`, and returns the base64 encoded signature as a string.
        *   `broadcast_command(command, duration=2.0)`: Constructs a specific payload containing `botId`, `timestamp`, `command` (e.g., "move_forward"), and `duration`. It generates the HMAC for this payload, wraps both payload and HMAC into a final JSON structure, encodes it, and sends it via UDP to all IPs in `TARGET_IPS`.
        *   `broadcast_info(payload)`: Takes a pre-constructed dictionary `payload`, generates its HMAC signature, wraps both payload and HMAC into a final JSON structure, encodes it, and sends it via UDP to all IPs in `TARGET_IPS`.

## Usage:

1.  Instantiate the `Broadcaster` class, providing the shared secret key (as bytes), a list of target IP addresses (strings), and a unique ID for the broadcaster bot.
    ```python
    secret = b'your_shared_secret_key'
    ips = ['192.168.1.101', '192.168.1.102']
    broadcaster = Broadcaster(secret, ips, "controller_bot")
    ```
2.  Call `broadcast_command` to send predefined commands:
    ```python
    broadcaster.broadcast_command("move_forward", duration=5.0)
    ```
3.  Call `broadcast_info` to send custom data payloads:
    ```python
    custom_data = {"botId": "sensor_hub", "timestamp": time.time(), "status": "ready", "value": 123}
    broadcaster.broadcast_info(custom_data)
    ```
4.  Ensure that receiver applications are running on the target IPs and listening on the correct UDP port (5005 by default) and are configured with the same `SECRET_KEY` for HMAC verification.

---

## Receiver.py

This script defines a `Receiving` class, structured as a ROS2 Node, intended to listen for UDP multicast messages on a specific group address and port. It verifies incoming messages using HMAC-SHA256 against a shared secret key. If verification succeeds and the payload contains sensor data, it processes and stores this data (sonar + lidar) as a NumPy vector in a Redis database. It also includes a method intended to publish received object location data to a ROS2 topic, although the publisher initialization and ROS2 node spinning logic are not present in the provided `main` function. The `main` function currently loads the secret key from a config file and runs the multicast listening loop directly.

## Dependencies:

*   `socket`
*   `struct`
*   `json`
*   `hashlib`
*   `base64`
*   `hmac`
*   `redis`
*   `numpy`
*   `os`
*   `rclpy.node` (Node)
*   `std_msgs.msg` (String)

## Classes:

*   **Receiving(Node):**
    *   **Attributes:**
        *   `SECRET_KEY` (bytes): The shared secret key for HMAC verification.
        *   `redis_client` (redis.Redis): An active connection to a Redis server (defaults to localhost:6379).
        *   `max_records` (int): The maximum number of sensor data records to keep per bot in Redis (default: 30).
        *   `publisher` (rclpy.publisher.Publisher): *Intended* attribute for publishing ROS2 messages. **Note:** Not initialized in the provided `__init__`. Needs to be added for `publish_object_location` to function correctly within a ROS2 context.
    *   **Methods:**
        *   `__init__(secret_key)`: Initializes the ROS2 node structure, stores the secret key, and connects to the Redis database.
        *   `verify_hmac(received_payload: dict, received_hmac: str) -> bool`: Verifies the received base64 HMAC string against a locally computed HMAC for the received payload dictionary, using the shared `SECRET_KEY`. Returns `True` if they match, `False` otherwise.
        *   `store_sensor_data(bot_id, sensors)`: Takes a bot ID and a dictionary of sensor readings (`sensors` assumed to contain "sonar" and "lidar" lists). Concatenates sonar and lidar data into a NumPy float32 array, converts it to bytes, and pushes it to a Redis list keyed by `bot:{bot_id}:data`. It also sets an expiration time and trims the list to `max_records`.
        *   `publish_object_location(bot_id, sensors)`: Constructs a dictionary containing the bot ID and sensor data, serializes it to a JSON string, wraps it in a `std_msgs.msg.String`, and publishes it using the `self.publisher`. **Note:** Requires `self.publisher` to be properly initialized (e.g., in `__init__`) with a topic like `/object_locations`.
        *   `receive()`: Enters an infinite loop to:
            1.  Set up a UDP socket, enable address reuse, and bind to the multicast port (5005).
            2.  Join the multicast group ("239.1.1.1").
            3.  Listen for incoming datagrams.
            4.  Decode the received data as UTF-8.
            5.  Attempt to parse the data as JSON, expecting "payload" and "hmac" keys.
            6.  Call `verify_hmac`. If successful, print the verified JSON and call `store_sensor_data` if "sensors" are present in the payload.
            7.  Print error/warning messages for HMAC failures or JSON parsing errors.

## Methods/Functions (Standalone):

*   **main():**
    1.  Attempts to load a configuration file from `~/shared_students/CapstoneFinalRepo/config.json`.
    2.  Extracts the `secret_key` from the config and encodes it to bytes.
    3.  Instantiates the `Receiving` class with the secret key.
    4.  Calls the `receiver.receive()` method directly, starting the indefinite listening loop.
    5.  Includes basic error handling for file not found, key errors, or other exceptions during config loading.
    *   **Note:** This `main` function does *not* initialize `rclpy` or use `rclpy.spin()`. It runs the `receive()` method's loop outside the standard ROS2 node execution model.

## Usage:

1.  Ensure Python 3 and required libraries (`redis`, `numpy`, `rclpy`) are installed.
2.  Ensure a Redis server is running and accessible (defaults to localhost:6379).
3.  Create a configuration file at `~/shared_students/CapstoneFinalRepo/config.json` containing at least the shared secret key:
    ```json
    {
      "secret_key": "your_shared_secret_key_here"
    }
    ```
4.  Run the script from the command line:
    ```bash
    python Receiver.py
    ```
5.  The script will start listening for UDP multicast messages on port 5005, group 239.1.1.1. It will verify incoming messages and store sensor data in Redis if applicable.
6.  To use the ROS2 publishing features, the script would need modification to initialize `rclpy` in `main`, create the publisher in `__init__`, and use `rclpy.spin(receiver)` in `main` instead of calling `receiver.receive()` directly.