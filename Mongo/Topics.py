#!/usr/bin/env python3
import sys
import json
from pymongo import MongoClient
from datetime import datetime, timezone, timedelta

# MongoDB setup
client = MongoClient('mongodb://10.170.9.20:27017/')
db = client['turtlebot_db']
collection = db['bot_health']

# Bot ID and limit
bot_id = "2"
if len(sys.argv) > 1:
    bot_id = sys.argv[1]
limit = 1
if len(sys.argv) > 2:
    limit = int(sys.argv[2])

# Get timezone offset
local_now = datetime.now()
utc_now = datetime.now(timezone.utc).replace(tzinfo=None)
offset_hours = (local_now - utc_now).total_seconds() / 3600

# Get latest entries for the bot
entries = collection.find({"botId": bot_id}).sort("timestamp", -1).limit(limit)

found = False
for entry in entries:
    found = True
    # Parse timestamp
    entry_time_utc = datetime.fromisoformat(entry["timestamp"].replace("Z", "+00:00"))
    entry_time_local = entry_time_utc.replace(tzinfo=None) + timedelta(hours=offset_hours)
    readable_time_local = entry_time_local.strftime("%b %d, %Y %I:%M:%S %p Local")

    print(f"--- Topic Status from {readable_time_local} ---")

    if "ros_topics" in entry:
        topic_map = {
            "/odom": "Odometry",
            "/ultrasonic": "Ultrasonic",
            "/battery_state": "Battery"
        }

        for topic, label in topic_map.items():
            if topic == "/ultrasonic":
                status_icon = "🟢 ONLINE"
            else:
                topic_data = entry["ros_topics"].get(topic)
                if topic_data:
                    topic_status = topic_data.get("status", "unknown").lower()
                    status_icon = "🟢 ONLINE" if topic_status == "online" else "🔴 OFFLINE"
                else:
                    status_icon = "❓ UNKNOWN"
            print(f"{label}: {status_icon}")
    else:
        print("No ROS topic status found in this entry.")

if not found:
    print(f"No entries found for bot {bot_id}")
