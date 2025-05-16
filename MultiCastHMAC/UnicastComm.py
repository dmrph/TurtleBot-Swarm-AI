import socket
import json
import hashlib
import base64
import hmac
import threading
import time
import redis
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UnicastComm(Node):
    def __init__(self, secret_key, bot_id, target_ips=None, use_redis=True, use_ros=True):
        """
        Initialize a combined unicast broadcaster and receiver.
        
        Args:
            secret_key: Secret key for HMAC generation/verification
            bot_id: ID of this bot
            target_ips: List of IP addresses to send broadcasts to (can be added later)
            use_redis: Whether to use Redis for storing sensor data
            use_ros: Whether to use ROS2 for publishing data
        """
        if use_ros:
            super().__init__('unicast_comm_node')
            self.publisher = self.create_publisher(String, '/object_locations', 10)
        
        self.SECRET_KEY = secret_key
        self.BOT_ID = bot_id
        self.UDP_PORT = 5005
        
        # Target IPs for unicast
        self.target_ips = target_ips or []
        
        # Redis setup (optional)
        self.use_redis = use_redis
        if use_redis:
            self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=False)
            self.max_records = 30
        
        self.use_ros = use_ros
        self._shutdown_flag = False
        self._receive_thread = None
        
        # Setup receiver socket
        self._setup_receiver_socket()
    
    def _setup_receiver_socket(self):
        """Setup the unicast receiver socket"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to all interfaces on the specified port
            self.sock.bind(("", self.UDP_PORT))
            
            # Make socket non-blocking for the receive loop
            self.sock.setblocking(False)
            
            if self.use_ros:
                self.get_logger().info(f"Unicast receiver socket setup on port {self.UDP_PORT}")
            else:
                print(f"Unicast receiver socket setup on port {self.UDP_PORT}")
        except Exception as e:
            if self.use_ros:
                self.get_logger().error(f"Error setting up receiver socket: {e}")
            else:
                print(f"Error setting up receiver socket: {e}")
    
    def add_target_ip(self, ip_address):
        """Add a target IP address for broadcasts"""
        if ip_address not in self.target_ips:
            self.target_ips.append(ip_address)
            if self.use_ros:
                self.get_logger().info(f"Added target IP: {ip_address}")
            else:
                print(f"Added target IP: {ip_address}")
    
    def remove_target_ip(self, ip_address):
        """Remove a target IP address"""
        if ip_address in self.target_ips:
            self.target_ips.remove(ip_address)
            if self.use_ros:
                self.get_logger().info(f"Removed target IP: {ip_address}")
            else:
                print(f"Removed target IP: {ip_address}")
    
    def generate_hmac(self, payload: dict) -> str:
        """Generate an HMAC signature for the given payload"""
        message = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        signature = hmac.new(self.SECRET_KEY, message, hashlib.sha256).digest()
        return base64.b64encode(signature).decode()
    
    def verify_hmac(self, received_payload: dict, received_hmac: str) -> bool:
        """Verify the HMAC signature of a received payload"""
        message = json.dumps(received_payload, separators=(',', ':')).encode('utf-8')
        expected_signature = hmac.new(self.SECRET_KEY, message, hashlib.sha256).digest()
        expected_hmac = base64.b64encode(expected_signature).decode()
        return hmac.compare_digest(received_hmac, expected_hmac)
    
    def broadcast_info(self, payload):
        """Send information via unicast UDP to all target IPs"""
        # Include botId if not already present
        if "botId" not in payload:
            payload["botId"] = self.BOT_ID
            
        # Add timestamp if not present
        if "timestamp" not in payload:
            payload["timestamp"] = time.time()
            
        # Generate HMAC signature
        hmac_signature = self.generate_hmac(payload)
        
        # Create the secure message
        secure_message = {
            "payload": payload,
            "hmac": hmac_signature
        }
        
        # Convert to bytes
        message_bytes = json.dumps(secure_message).encode('utf-8')
        
        try:
            # Create a separate socket for sending
            send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Send the message to each target IP
            if not self.target_ips:
                if self.use_ros:
                    self.get_logger().warning("No target IPs configured for broadcast")
                else:
                    print("⚠️ No target IPs configured for broadcast")
            
            successful_sends = 0
            for ip in self.target_ips:
                try:
                    send_sock.sendto(message_bytes, (ip, self.UDP_PORT))
                    successful_sends += 1
                except Exception as e:
                    if self.use_ros:
                        self.get_logger().error(f"Failed to send to {ip}: {e}")
                    else:
                        print(f"❌ Failed to send to {ip}: {e}")
            
            if successful_sends > 0:
                if self.use_ros:
                    self.get_logger().info(f"Sent to {successful_sends}/{len(self.target_ips)} targets")
                else:
                    print(f"🔹 Sent to {successful_sends}/{len(self.target_ips)} targets")
                
        except Exception as e:
            if self.use_ros:
                self.get_logger().error(f"Failed to send broadcast: {e}")
            else:
                print(f"❌ Failed to send broadcast: {e}")
        finally:
            send_sock.close()
    
    def store_sensor_data(self, bot_id, sensors):
        """Store sensor data in Redis (if enabled)"""
        if not self.use_redis:
            return
            
        try:
            vector_data = np.array(sensors["sonar"] + sensors["lidar"], dtype=np.float32)
            bot_key = f"bot:{bot_id}:data"
            self.redis_client.rpush(bot_key, vector_data.tobytes())
            self.redis_client.expire(bot_key, 3600)
            self.redis_client.ltrim(bot_key, -self.max_records, -1)
            
            if self.use_ros:
                self.get_logger().info(f"Stored bot {bot_id} sensor data in Redis")
            else:
                print(f"Stored bot {bot_id} sensor data in Redis")
        except Exception as e:
            if self.use_ros:
                self.get_logger().error(f"Error storing in Redis: {e}")
            else:
                print(f"Error storing in Redis: {e}")
    
    def publish_object_location(self, bot_id, sensors):
        """Publish sensor data to ROS2 topic (if enabled)"""
        if not self.use_ros:
            return
            
        try:
            message = {
                "bot_id": bot_id,
                "sensors": sensors
            }
            ros_message = String()
            ros_message.data = json.dumps(message)
            self.publisher.publish(ros_message)
            self.get_logger().info(f"Published object location for bot {bot_id}")
        except Exception as e:
            self.get_logger().error(f"Error publishing to ROS2: {e}")
    
    def process_received_data(self, json_data, addr):
        """Process received JSON data"""
        received_payload = json_data.get("payload")
        received_hmac = json_data.get("hmac")
        
        if not (received_payload and received_hmac):
            if self.use_ros:
                self.get_logger().warning(f"Malformed JSON from {addr}")
            else:
                print(f"⚠️ Malformed JSON from {addr}")
            return
            
        # Verify HMAC
        if not self.verify_hmac(received_payload, received_hmac):
            if self.use_ros:
                self.get_logger().warning(f"HMAC verification failed from {addr}")
            else:
                print(f"❌ HMAC verification failed from {addr}")
            return
            
        # Successfully verified
        if self.use_ros:
            self.get_logger().info(f"Verified JSON from {addr}")
        else:
            print(f"\n✅ Verified JSON from {addr}")
            print(json.dumps(received_payload, indent=4))
        
        # Check if this is from our own bot (to avoid processing our own broadcasts)
        bot_id = received_payload.get("botId")
        if bot_id == self.BOT_ID:
            if self.use_ros:
                self.get_logger().debug(f"Ignoring our own broadcast (bot_id: {bot_id})")
            return
            
        # Add the sender's IP to our target list if not already there
        sender_ip = addr[0]
        if sender_ip not in self.target_ips:
            self.add_target_ip(sender_ip)
        
        # Process sensor data if present
        if "sensors" in received_payload and bot_id:
            # Store in Redis
            if self.use_redis:
                self.store_sensor_data(bot_id, received_payload["sensors"])
            
            # Publish to ROS2
            if self.use_ros:
                self.publish_object_location(bot_id, received_payload["sensors"])
    
    def _receive_loop(self):
        """Background thread to continuously receive unicast messages"""
        if self.use_ros:
            self.get_logger().info("Starting receiver loop in background thread")
        else:
            print("Starting receiver loop in background thread")
            
        while not self._shutdown_flag:
            try:
                data, addr = self.sock.recvfrom(4096)
                decoded_data = data.decode("utf-8")
                
                try:
                    json_data = json.loads(decoded_data)
                    self.process_received_data(json_data, addr)
                except json.JSONDecodeError:
                    if self.use_ros:
                        self.get_logger().warning(f"Invalid JSON from {addr}")
                    else:
                        print(f"⚠️ Invalid JSON from {addr}")
                        
            except BlockingIOError:
                # No data available yet (normal with non-blocking sockets)
                time.sleep(0.01)
            except Exception as e:
                if self.use_ros:
                    self.get_logger().error(f"Error in receive loop: {e}")
                else:
                    print(f"❌ Error in receive loop: {e}")
                time.sleep(0.1)
        
        # Clean up when thread exits
        if self.sock:
            self.sock.close()
            
        if self.use_ros:
            self.get_logger().info("Receiver thread stopped")
        else:
            print("Receiver thread stopped")
    
    def start(self):
        """Start the background receiver thread"""
        if self._receive_thread is None or not self._receive_thread.is_alive():
            self._shutdown_flag = False
            self._receive_thread = threading.Thread(target=self._receive_loop)
            self._receive_thread.daemon = True
            self._receive_thread.start()
            
            if self.use_ros:
                self.get_logger().info("Background receiver thread started")
            else:
                print("Background receiver thread started")
        else:
            if self.use_ros:
                self.get_logger().warning("Receiver thread already running")
            else:
                print("Receiver thread already running")
    
    def stop(self):
        """Stop the background receiver thread"""
        if self._receive_thread and self._receive_thread.is_alive():
            self._shutdown_flag = True
            self._receive_thread.join(timeout=2.0)
            
            if self.use_ros:
                self.get_logger().info("Receiver thread stopped")
            else:
                print("Receiver thread stopped")
                
    def __del__(self):
        """Cleanup when the object is destroyed"""
        self.stop()
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()