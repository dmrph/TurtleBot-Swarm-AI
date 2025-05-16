import socket
import struct
import json
import hashlib
import base64
import hmac
import redis
import numpy as np
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type for publishing string messages.

CONFIG_PATH = os.path.expanduser('~/shared_students/CapstoneFinalRepo/config.json')

class Receiver(Node):  # Changed class name from Receiving to Receiver to match usage in main()
    def __init__(self, secret_key):  # Constructor method to initialize the Receiver node.
        super().__init__('receiver_node')  # Initialize the node with the name 'receiver_node'.
        self.SECRET_KEY = secret_key  # Store the secret key for HMAC verification.
        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=False)  # Connect to a Redis database.
        self.max_records = 30  # Set the maximum number of records to store in Redis.
        # Create a publisher for object location data
        self.publisher = self.create_publisher(String, '/object_locations', 10)  # Added missing publisher initialization

    def verify_hmac(self, received_payload: dict, received_hmac: str) -> bool:
        message = json.dumps(received_payload, separators=(',', ':')).encode('utf-8')
        expected_signature = hmac.new(self.SECRET_KEY, message, hashlib.sha256).digest()
        expected_hmac = base64.b64encode(expected_signature).decode()
        return hmac.compare_digest(received_hmac, expected_hmac)

    def store_sensor_data(self, bot_id, sensors):
        vector_data = np.array(sensors["sonar"] + sensors["lidar"], dtype=np.float32)
        bot_key = f"bot:{bot_id}:data"
        self.redis_client.rpush(bot_key, vector_data.tobytes())
        self.redis_client.expire(bot_key, 3600)
        self.redis_client.ltrim(bot_key, -self.max_records, -1)
        print(f"Stored bot {bot_id} sensor vector in Redis.")

    def publish_object_location(self, bot_id, sensors):  # Method to publish object location data to a ROS2 topic.
        # Create a message with object location data
        message = {
            "bot_id": bot_id,  # Include the bot ID in the message.
            "sensors": sensors  # Include the sensor data in the message.
        }
        ros_message = String()  # Create a ROS2 String message.
        ros_message.data = json.dumps(message)  # Serialize the message as a JSON string.
        self.publisher.publish(ros_message)  # Publish the message to the '/object_locations' topic.
        self.get_logger().info(f"Published object location for bot {bot_id}")  # Log the publication event.
    
    def receive(self):
        MULTICAST_GROUP = "239.1.1.1"
        UDP_PORT = 5005

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", UDP_PORT))

        mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        print("Listening for JSON multicast messages...")

        while True:
            try:
                data, addr = sock.recvfrom(4096)
                decoded_data = data.decode("utf-8")

                try:
                    json_data = json.loads(decoded_data)
                    received_payload = json_data.get("payload")
                    received_hmac = json_data.get("hmac")

                    if received_payload and received_hmac:
                        if self.verify_hmac(received_payload, received_hmac):
                            print(f"\n✅ Verified JSON from {addr}:")
                            print(json.dumps(json_data, indent=4))

                            if "sensors" in received_payload:
                                bot_id = received_payload.get("botId")  # Using get() to avoid KeyError
                                if bot_id:
                                    self.store_sensor_data(bot_id, received_payload["sensors"])
                                    # Also publish the data to ROS2 topic
                                    self.publish_object_location(bot_id, received_payload["sensors"])
                                else:
                                    print("⚠️ Missing botId in payload")
                        else:
                            print(f"❌ HMAC verification failed from {addr}")
                    else:
                        print(f"⚠️ Malformed JSON (missing 'payload' or 'hmac') from {addr}")

                except json.JSONDecodeError:
                    print(f"⚠️ Invalid JSON from {addr}: {decoded_data}")

            except Exception as e:
                print(f"❌ Error receiving data: {e}")

def main():
    # Initialize ROS2
    rclpy.init()
    
    try:
        with open(CONFIG_PATH, 'r') as config_file:
            config = json.load(config_file)
        secret_key = config['secret_key'].encode()

        receiver = Receiver(secret_key)
        receiver.receive()

    except FileNotFoundError:
        print(f"❌ Config file not found at: {CONFIG_PATH}")
    except KeyError:
        print(f"❌ 'secret_key' not found in config file.")
    except Exception as e:
        print(f"❌ Error loading config: {e}")
    finally:
        # Shutdown ROS2 when exiting
        rclpy.shutdown()

if __name__ == "__main__":
    main()