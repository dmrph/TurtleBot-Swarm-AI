# CAPSTONE 2025

Mission: Program a group of TurtleBot3 Burger-model robots to operate in a swarm-style formation to find predetermined charms while avoiding obstacles. The TurtleBots are equipped with a Raspberry Pi, LiDAR and ultrasonic sensors, a web camera for computer vision, and an LCD display. The TurtleBots work with an AI agents system, where different components run asynchronously to perform the multitude of tasks that need to run together.


## Swarm Algorithm

The TurtleBots use a blend of particle swarm optimization and bee colony optimization in order to travel and identify charms while avoiding obstacles.

Particle Swarm Optimization (PSO): An algorithm for finding the global minimum of a function; in this case, the algorithm has each individual calculate and share the distance to their own closest charm, compile the information, and decide on a global best charm to pursue.

Bee Colony Optimization (BCO): The bee colony algorithm has the bots perform a local exploration of their area and checks if a charm is within their threshold. It then pathfinds to a charm, optimizing its path to avoid obstacles.

## Multicasting with HMAC

The TurtleBots communicate on a designated UDP port, sending and receiving messages containing essential information such as location information and detection of objects and charms. Messages are encoded using HMAC (Hash-based Message Authentication Code) to ensure integrity and security of communication.

## MongoDB and Redis Server

The MongoDB is used for storing logs of the TurtleBots. Information saved includes their current global position, detection of objects and charms, and sensor information.

Redis is a database that works directly in memory. The Redis servers on the TurtleBots are used to save position information and object detection.

## AprilTags and YOLO Object Detection

AprilTags are used for identifying the individual bots and base station. Each TurtleBot is equipped with 3 AprilTags, while the base station features 4, one for each cardinal direction.

YOLO (You Only Look Once) is an object-detection system. The TurtleBots use this system to identify charms and obstacles that they detect. The TurtleBots are capable of detecting theiir surroundings using their sensors; once an obstacle is found, they can approach it and attempt to identify it using YOLO.
