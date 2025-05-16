import time
import redis
import numpy as np

class Redis_Ops:
    redis_client = redis.Redis(host='localhost', port=6379, decode_responses=False)

    @staticmethod
    def cosine_similarity(v1, v2):
        return np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))

    @classmethod
    def log_redis_data(cls, obj_id, objXY, objOrient):
        timestamp = time.time()
        data = [timestamp, objXY[0], objXY[1], objOrient]
        vector_data = np.array(data, dtype=np.float32)

        # Store vector data in Redis
        cls.redis_client.set(obj_id, vector_data.tobytes())
        print(f"Added Object: {obj_id} with vector: {vector_data}")

    @classmethod
    def clear_redis_data(cls):
        cls.redis_client.flushall()

    @classmethod
    def retrieve_redis_data(cls, key):
        data = cls.redis_client.get(key)
        return np.frombuffer(data, dtype=np.float32) if data else None

    @classmethod
    def retrieve_closest_vector(cls, v1): # returns vector from db with highest cosine similarity with v1
        keys = cls.redis_client.keys("*")
        best_score = -1
        best_key = None

        for key in keys:
            raw = cls.redis_client.get(key)
            if not raw:
                continue
            v2 = np.frombuffer(raw, dtype=np.float32)
            score = cls.cosine_similarity(v1, v2)
            if score > best_score:
                best_score = score
                best_key = key

        data = cls.redis_client.get(best_key)
        return best_key, np.frombuffer(data, dtype=np.float32) if data else None
