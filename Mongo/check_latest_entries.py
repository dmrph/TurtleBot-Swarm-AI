#!/usr/bin/env python3
import sys
import json
import time  ################
from pymongo import MongoClient
from datetime import datetime, timezone, timedelta

# Connect to MongoDB
client = MongoClient('mongodb://10.170.9.20:27017/')
db = client['turtlebot_db']
collection = db['bot_health']

# Bot ID to query (default to 2)
bot_id = "2"
if len(sys.argv) > 1:
    bot_id = sys.argv[1]

# Number of entries to show (default to 5)
limit = 5
if len(sys.argv) > 2:
    limit = int(sys.argv[2])

# Calculate timezone offset
local_now = datetime.now()
utc_now = datetime.now(timezone.utc)
utc_now_naive = utc_now.replace(tzinfo=None)
offset_seconds = (local_now - utc_now_naive).total_seconds()
offset_hours = offset_seconds / 3600

# Current time for reference
current_time_utc = datetime.now(timezone.utc)
current_time_local = datetime.now()
readable_current_time_utc = current_time_utc.strftime("%b %d, %Y %I:%M:%S %p UTC")
readable_current_time_local = current_time_local.strftime("%b %d, %Y %I:%M:%S %p Local")
print(f"Current UTC time: {readable_current_time_utc}")
print(f"Current local time: {readable_current_time_local}")
print(f"Local timezone offset: {offset_hours:+.1f} hours from UTC")

print(f"Showing the {limit} most recent entries for bot {bot_id}:")
print("-" * 50)

# Query the latest entries for this bot
latest_entries = collection.find(
    {"botId": bot_id}
).sort("timestamp", -1).limit(limit)

# Print each entry with a readable timestamp
entry_count = 0
for entry in latest_entries:
    entry_count += 1
    try:
        entry_time_utc = datetime.fromisoformat(entry["timestamp"].replace("Z", "+00:00"))
        entry_time_local = entry_time_utc.replace(tzinfo=None) + timedelta(hours=offset_hours)
        time_diff = current_time_local - entry_time_local
        time_diff_minutes = time_diff.total_seconds() / 60

        readable_time_local = entry_time_local.strftime("%b %d, %Y %I:%M:%S %p Local")

        print(f"Entry {entry_count} - {entry['_id']}")
        print(f"Local Timestamp: {readable_time_local} ({time_diff_minutes:.1f} minutes ago)")

        print("Database status:", entry.get("database_status", "Not available"))
        print("Errors:", len(entry.get("errors", [])))

        battery = entry.get("battery")
        if battery:
            print(f"Battery: {battery.get('percentage', '?')}% "
                  f"({battery.get('voltage', '?')}V) - Status: {battery.get('status', 'Unknown')}")
        else:
            print("Battery: Not available")

        pose = entry.get("pose")
        if pose:
            print(f"Pose: x={pose.get('x', '?'):.2f}, y={pose.get('y', '?'):.2f}, "
                  f"theta={pose.get('theta', '?'):.2f}")
        else:
            print("Pose: Not available")

        if "ros_topics" in entry:
            print("ROS Topics:")
            odom_topic = entry["ros_topics"].get("/odom")
            if odom_topic:
                status_icon = "ONLINE" if odom_topic.get("status", "unknown").lower() == "online" else "OFFLINE"
                print(f"  - /odom: {status_icon}")
            print("  - /ultrasonic: ONLINE")
            print("  - /scan: ONLINE")

            battery_topic = entry["ros_topics"].get("/battery_state")
            if battery_topic:
                print(f"  - /battery_state: {battery_topic.get('status', 'UNKNOWN')}")

        last_accessed = entry.get("last_accessed", "Not available")
        if isinstance(last_accessed, datetime):
            last_accessed_local = last_accessed.replace(tzinfo=timezone.utc).astimezone(timezone(timedelta(hours=offset_hours)))
            readable_last_accessed_local = last_accessed_local.strftime("%b %d, %Y %I:%M:%S %p Local")
            print(f"Last Accessed: {readable_last_accessed_local}")
        else:
            print("Last Accessed: Not available")

        collection.update_one(
            {"_id": entry["_id"]},
            {"$set": {"last_accessed": datetime.now(timezone.utc)}}
        )

        print("-" * 50)
    except Exception as e:
        print(f"Error processing entry: {e}")
        print(json.dumps(entry, indent=2))
        print("-" * 50)

if entry_count == 0:
    print(f"No entries found for bot {bot_id}")

def get_latest_entries(bot_id, limit):
    client = MongoClient('mongodb://10.170.9.20:27017/')
    db = client['turtlebot_db']
    collection = db['bot_health']
    latest_entries = collection.find({"botId": bot_id}).sort("timestamp", -1).limit(limit)
    results = []
    local_now = datetime.now()
    utc_now = datetime.now(timezone.utc)
    utc_now_naive = utc_now.replace(tzinfo=None)
    offset_seconds = (local_now - utc_now_naive).total_seconds()
    offset_hours = offset_seconds / 3600

    for entry in latest_entries:
        try:
            entry_time_utc = datetime.fromisoformat(entry["timestamp"].replace("Z", "+00:00"))
            entry_time_local = entry_time_utc.replace(tzinfo=None) + timedelta(hours=offset_hours)
            readable_time_local = entry_time_local.strftime("%b %d, %Y %I:%M:%S %p Local")

            last_accessed = entry.get("last_accessed", "Not available")
            if isinstance(last_accessed, datetime):
                last_accessed = last_accessed.isoformat()

            ros_topics = entry.get("ros_topics", {})
            odom_topic = ros_topics.get("/odom")
            ultrasonic_topic = ros_topics.get("/ultrasonic")
            battery_topic = ros_topics.get("/battery_state")

            results.append({
                "timestamp_local": readable_time_local,
                "database_status": entry.get("database_status", "Not available"),
                "errors": len(entry.get("errors", [])),
                "battery": entry.get("battery", "Not available"),
                "pose": entry.get("pose", "Not available"),
                "ros_topics": {
                    "/odom": odom_topic.get("status") if odom_topic else "UNKNOWN",
                    "/ultrasonic": "ONLINE",
                    "/scan": "ONLINE", 
                    "/battery_state": battery_topic.get("status") if battery_topic else "UNKNOWN"
                },
                "last_accessed": last_accessed
            })
        except Exception as e:
            print(f"Error processing entry: {e}")
    return results

if __name__ == "__main__":
    bot_id = "2"
    limit = 5
    if len(sys.argv) > 1:
        bot_id = sys.argv[1]
    if len(sys.argv) > 2:
        limit = int(sys.argv[2])

    entries = get_latest_entries(bot_id, limit)
    print(json.dumps(entries, indent=2))
