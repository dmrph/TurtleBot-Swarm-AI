#!/usr/bin/env python3
from pymongo import MongoClient
from datetime import datetime, timezone
import json
import random
import base64
import time
from threading import Lock

class TurtleBotMongoLogger:
    def __init__(self, bot_id, mongo_uri='mongodb://10.32.34.141:27017/', 
                 db_name='turtlebot_db', collection_name='bot_logs'):
        
        """Initialize the TurtleBot MongoDB Logger"""
        self.client = MongoClient(mongo_uri)
        self.db = self.client[db_name]
        self.collection = self.db[collection_name]
        self.bot_id = bot_id
        
        # Data storage
        self.location = {"x": 0.0, "y": 0.0, "orientation": 0.0}
        self.sonar_data = []
        self.lidar_data = []
        self.camera_data = ""
        
        # Thread safety
        self.data_lock = Lock()
        
        print(f"Logger initialized for {bot_id}")
    
    def generate_mock_data(self):
        """Generate all mock data at once"""
        with self.data_lock:
            # Generate position data
            self.location["x"] = round(random.uniform(0, 100), 2)
            self.location["y"] = round(random.uniform(0, 100), 2)
            self.location["orientation"] = round(random.uniform(0, 359), 1)
            
            # Generate sensor data
            self.lidar_data = [round(random.uniform(0.1, 10.0), 2) for _ in range(10)]
            
            # Update sonar data
            new_reading = round(random.uniform(0.1, 5.0), 2)
            self.sonar_data.append(new_reading)
            if len(self.sonar_data) > 5:
                self.sonar_data.pop(0)
                
            # Mock camera data
            mock_image = f"camera_data_{int(time.time())}"
            self.camera_data = base64.b64encode(mock_image.encode()).decode()
    
    def log_data(self):
        """Log the current state to MongoDB"""
        with self.data_lock:
            # Create log entry following the schema
            log_data = {
                "botId": self.bot_id,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "location": {
                    "x": self.location["x"],
                    "y": self.location["y"],
                    "orientation": self.location["orientation"]
                },
                "sensors": {
                    "sonar": self.sonar_data.copy(),
                    "lidar": self.lidar_data.copy(),
                    "camera": self.camera_data
                }
            }
        
        try:
            # Insert into MongoDB
            result = self.collection.insert_one(log_data)
            print(f"Log created: ID {result.inserted_id}")
            return result.inserted_id
        except Exception as e:
            print(f"Error: {e}")
            return None
    
    def retrieve_logs(self, limit=10):
        """Retrieve recent logs for this bot"""
        query = {"botId": self.bot_id}
        try:
            logs = self.collection.find(query).sort("timestamp", -1).limit(limit)
            return list(logs)
        except Exception as e:
            print(f"Error retrieving logs: {e}")
            return []

if __name__ == '__main__':
    try:
        # turtle bot ID
        bot_id = "2"
        
        # Create logger
        logger = TurtleBotMongoLogger(bot_id)
        print(f"MongoDB Logger started for TurtleBot {bot_id}")
        
        # Log mock data for 5 iterations
        for i in range(5):
            logger.generate_mock_data()
            logger.log_data()
            print(f"Log #{i+1} created")
            time.sleep(1)
        
        # Retrieve and display logs
        print("\nRecent logs:")
        recent_logs = logger.retrieve_logs(limit=3)
        for i, log in enumerate(recent_logs):
            log["_id"] = str(log["_id"])
            print(f"\nLog {i+1}:")
            print(json.dumps(log, indent=2))
            
        print("\nDone!")
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")