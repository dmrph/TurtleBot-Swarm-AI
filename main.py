import sys
import rclpy
import traceback
import signal
import asyncio

# Import all components from the existing module
from SwarmAI.object_avoid_testing_sonic import *

is_shutting_down = False
object_detected = False
charm_detected = False
charm_approach_in_progress = False

def local_handle_shutdown(signum, frame):
    """Local handler to ensure proper shutdown"""
    global is_shutting_down, charm_approach_in_progress
    print("\nShutdown signal received in main.py! Stopping the robot...")
    is_shutting_down = True
    charm_approach_in_progress = False
    sys.exit(0)

if __name__ == "__main__":
    # Set up signal handlers
    signal.signal(signal.SIGINT, local_handle_shutdown)
    signal.signal(signal.SIGTERM, local_handle_shutdown)

    try:
        print("Starting robot control system...")

        # Run the main function from the imported module
        asyncio.run(main())

    except KeyboardInterrupt:
        print("Keyboard interrupt detected in main.py")
        is_shutting_down = True
        charm_approach_in_progress = False

    except Exception as e:
        print(f"Error in main.py: {e}")
        print("Traceback:")
        traceback.print_exc()

    finally:
        print("Cleaning up and shutting down...")
        is_shutting_down = True
        charm_approach_in_progress = False

        # Make sure rclpy is shutdown if it was initialized
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        sys.exit(0)
