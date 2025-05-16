#!/usr/bin/env python3
from pymongo import MongoClient
from datetime import datetime
import json
import base64
import sys

# MongoDB connection information
#MONGO_URI = 'mongodb://10.32.34.141:27017/'
MONGO_URI = 'mongodb://10.14.14.109:27017/'
DB_NAME = 'turtlebot_db'
COLLECTION_NAME = 'bot_logs'

def test_mongodb_connection():
    """Test the MongoDB connection and insert a test document"""
    try:
        # Connect to MongoDB
        client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=5000)
        
        # Force a connection to verify it works
        client.server_info()
        print(f" Successfully connected to MongoDB at {MONGO_URI}")
        
        # Get database and collection
        db = client[DB_NAME]
        collection = db[COLLECTION_NAME]
        
        # Create a test document
        test_doc = {
            "botId": "test_bot",
            "timestamp": datetime.utcnow().isoformat(),
            "location": {
                "x": 1.23,
                "y": 4.56,
                "orientation": 90.0
            },
            "sensors": {
                "sonar": [1.1, 2.2, 3.3, 4.4, 5.5],
                "lidar": [0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5],
                "camera": base64.b64encode(b"test_camera_data").decode()
            }
        }
        
        # Insert the test document
        result = collection.insert_one(test_doc)
        print(f"✅ Successfully inserted test document with ID: {result.inserted_id}")
        
        # Retrieve the document to verify it was stored correctly
        retrieved_doc = collection.find_one({"_id": result.inserted_id})
        if retrieved_doc:
            # Convert ObjectId to string for display
            retrieved_doc["_id"] = str(retrieved_doc["_id"])
            print("\nRetrieved document:")
            print(json.dumps(retrieved_doc, indent=2))
            print("\n✅ Document retrieval successful")
        
        # Delete the test document
        collection.delete_one({"_id": result.inserted_id})
        print("✅ Test document deleted successfully")
        
        return True
        
    except Exception as e:
        print(f" Error: {e}")
        return False
    finally:
        if 'client' in locals():
            client.close()
            print("MongoDB connection closed")

if __name__ == "__main__":
    print("Testing MongoDB Connection and Schema...")
    success = test_mongodb_connection()
    if success:
        print("\n All tests passed! Your MongoDB setup is working correctly.")
        sys.exit(0)
    else:
        print("\n Tests failed. Please check your MongoDB configuration.")
        sys.exit(1)