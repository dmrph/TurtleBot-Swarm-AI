import socket
import json
import hmac
import hashlib
import base64
import time

class Broadcaster:
    def __init__(self, secret_key, bot_id):
        self.SECRET_KEY = secret_key
        self.UDP_PORT = 5005  # Ensure receivers listen on this port
        self.BOT_ID = bot_id
    
    def generate_hmac(self, payload: dict) -> str:
        # Generate an HMAC signature for the given payload.
        message = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        signature = hmac.new(self.SECRET_KEY, message, hashlib.sha256).digest()
        return base64.b64encode(signature).decode()
        

    def broadcast_info(self, payload):
        # Create the payload structure with the passed information
        
        hmac_signature = self.generate_hmac(payload) 

        secure_message = {
            "payload": payload,
            "hmac": hmac_signature
        }

        # Convert JSON payload to bytes
        MESSAGE = json.dumps(secure_message).encode('utf-8')

        # Multicast settings
        MULTICAST_GROUP = "239.1.1.1"
        MULTICAST_PORT = 5005

        try:
            # Create a UDP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

            # Set multicast Time-To-Live (TTL)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

            # Prevent the sender from receiving its own multicast packets
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

            # Send the message to the multicast group
            sock.sendto(MESSAGE, (MULTICAST_GROUP, MULTICAST_PORT))
            print(f"🔹 JSON payload sent to multicast group {MULTICAST_GROUP}:{MULTICAST_PORT}")

        except Exception as e:
            print(f"❌ Failed to send multicast message: {e}")

        finally:
            sock.close()
