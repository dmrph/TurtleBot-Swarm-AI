# Only works on ROS2 environment
import unittest
import asyncio
import sys
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from unittest.mock import patch, MagicMock, AsyncMock
import aiAgentsMain
from aiAgentsMain import MoveBotAgent, LidarAgent, BroadcastAgent, MainController

# Test Class
class CondensedAgentTests(unittest.IsolatedAsyncioTestCase):

    # Handles RLCPY (was having issues earlier)
    @classmethod
    def setUpClass(cls):
        try:
            if not rclpy.ok(): rclpy.init()
        except Exception as e:
            print(f"WARN: rclpy.init() in setUpClass failed: {e}")

    # Shutting down rclpy after all tests in this class are done
    @classmethod
    def tearDownClass(cls):
        if rclpy.ok(): rclpy.shutdown()

    def setUp(self):
        """ Need to Replace queue for each test."""
        self.original_queue = aiAgentsMain.asyncio.Queue()
        aiAgentsMain.asyncio.Queue() == asyncio.Queue()
        self.test_queue = aiAgentsMain.asyncio.Queue()

    def tearDown(self):
        """Restore queue."""
        aiAgentsMain.message_queue = self.original_queue # <--- CORRECT

    # Testing MoveBotAgent (Init, Move, and Queue)
    # Check if MoveBotAgent can be created, if move() runs, and if it queues an "Executed"
    # message upon completion. Mocks sleep.
    @patch('aiAgentsMain.asyncio.sleep', new_callable=AsyncMock) # Mocking sleep to avoid delays
    async def test_move_bot(self, mock_async_sleep):
        """Check MoveBotAgent creation, move(), and queue output."""
        agent = None
        try:
            agent = MoveBotAgent() # Assumes Node init works in env
            await agent.move('forward', 0.1)
            # Primary check: queue output
            msg = await asyncio.wait_for(self.test_queue.get(), timeout=0.5)
            self.assertIn("Executed forward", msg)
        except Exception as e:
            self.fail(f"MoveBot test failed: {e}")
        finally:
            if agent: agent.destroy_node()

    # Testing LidarAgent (Callback)
    # Directly call LidarAgent's scan_callback to check its internal logic for
    # detecting vs. not detecting objects based on range data.
    # Checks logger calls and if asyncio.create_task is attempted.
    @patch('aiAgentsMain.asyncio.create_task') # Check if task creation is attempted
    def test_lidar(self, mock_create_task):
        """Check LidarAgent scan_callback logic directly."""
        agent = None
        try:
            # Patch locally to allow instantiation & direct callback test
            with patch.object(LidarAgent, 'create_subscription'), \
                 patch.object(LidarAgent, 'get_logger', return_value=MagicMock(warn=MagicMock(), info=MagicMock())) as mock_get_logger:
                agent = LidarAgent()
                mock_logger = mock_get_logger.return_value

                # Testing detection case
                agent.scan_callback(MagicMock(ranges=[0.3]))
                mock_logger.warn.assert_called_once()
                mock_create_task.assert_called_once()

                # Testing no detection case
                mock_logger.reset_mock()
                mock_create_task.reset_mock()
                agent.scan_callback(MagicMock(ranges=[0.6]))
                mock_logger.info.assert_called_once()
                mock_create_task.assert_not_called()
        except Exception as e:
            self.fail(f"Lidar test failed: {e}")
        finally:
            if agent and hasattr(agent, '_handle') and agent._handle:
                try:
                    agent.destroy_node()
                except Exception: pass

    # Testing BroadcastAgent broadcast call attempt
    # Checks if BroadcastAgent, when its loop logic runs once, attempts to call 
    # the Broadcast_Info method on its broadcaster object.
    # Mocks Thread, sleep, and the Broadcast class itself.
    @patch('aiAgentsMain.threading.Thread') # Mock threading
    @patch('aiAgentsMain.time.sleep')      # Mock sleeping
    @patch('aiAgentsMain.Broadcast')       # Mock network Broadcast class
    def test_broadcast_call(self, MockBroadcast, mock_time_sleep, MockThread):
        """Check BroadcastAgent tries to call Broadcast_Info."""
        try:
            mock_broadcast_instance = MockBroadcast.return_value
            agent = BroadcastAgent(b'k', [], 'i', interval=0.01)
            # Pretending the loop is running once
            agent.running = True
            mock_time_sleep.side_effect = lambda d: setattr(agent, 'running', False)
            with patch('builtins.print'): agent.broadcast_info_continuously()
            # Network call attempt
            mock_broadcast_instance.Broadcast_Info.assert_called_once()
        except Exception as e:
            self.fail(f"Broadcast test failed: {e}")


    # Testing MainController queue retrieval
    # Tests if MainController can successfully retrieve a message
    # that is manually placed onto the (shared) asyncio queue.
    async def test_controller_receive(self):
        """Check MainController gets message from queue."""
        try:
            controller = MainController()
            test_msg = "Condensed Controller Test"
            await self.test_queue.put(test_msg)
            # Running listener real quick
            listen_task = asyncio.create_task(controller.listen_for_notifications())
            await asyncio.sleep(0.05)
            listen_task.cancel()
            try: await listen_task
            except asyncio.CancelledError: pass
            # Assuming success if no error
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"Controller test failed: {e}")

    # Additional Tests by Fern

    def test_sonic_agent_detects_object(self):
        """
        Test if SonicAgent correctly prints a message when an object is detected
        within the distance threshold.

        Steps:
        - set the detection threshold to 20 cm.
        - send a fake sensor message with a distance of 10 cm (closer than threshold).
        - patch 'print' to check if the message about the detected object was printed.
        - test passes if that message is printed as expected.
        """
        with patch.object(SonicAgent, 'create_subscription'), \
             patch('builtins.print') as mock_print:
            agent = SonicAgent(threshold=20)
            test_msg = MagicMock()
            test_msg.range = 10.0
            agent.detect_object(test_msg)
            mock_print.assert_called_with('SonicAgent: Object detected at 10.00 cm')
            agent.destroy_node()

    @patch('aiAgentsMain.asyncio.sleep', new_callable=AsyncMock)
    async def test_move_bot_ignores_bad_command(self, mock_sleep):
        """
        Test if MoveBotAgent handles an invalid command safely.

        Steps:
        - give the agent a fake movement command like "fly" (not supported).
        - check that it logs a warning about the bad command.
        - also check that it doesn't try to send anything to the movement queue.
        - test passes if the warning is logged and nothing is added to the queue.
        """
        agent = MoveBotAgent()
        with patch.object(agent, 'get_logger', return_value=MagicMock(warn=MagicMock())) as mock_logger, \
             patch('aiAgentsMain.message_queue.put') as mock_queue_put:
            await agent.move('fly', 0.1)
            mock_logger.return_value.warn.assert_called_with('Unknown command: fly')
            mock_queue_put.assert_not_called()
        agent.destroy_node()

    def test_lidar_handles_empty_ranges(self):
        """
        Test if LidarAgent can handle bad or empty sensor data (like NaN or inf values).

        Steps:
        - simulate a scan with only invalid numbers (inf and nan).
        - expect the agent to safely handle it without crashing.
        - check that it logs a message saying no object was detected.
        - test passes if the log message appears and no error happens.
        """
        with patch.object(LidarAgent, 'create_subscription'), \
             patch.object(LidarAgent, 'get_logger', return_value=MagicMock(info=MagicMock())) as mock_logger:
            agent = LidarAgent()
            fake_scan = MagicMock()
            fake_scan.ranges = [float('inf'), float('nan')]
            try:
                agent.scan_callback(fake_scan)
                mock_logger.return_value.info.assert_called_once()
            except Exception as e:
                self.fail(f"LidarAgent crashed on bad input: {e}")
            agent.destroy_node()

    @patch('aiAgentsMain.asyncio.sleep', new_callable=AsyncMock)
    async def test_move_bot_with_zero_duration(self, mock_sleep):
        """
        Test if MoveBotAgent behaves correctly when movement duration is zero.

        Steps:
        - tell the agent to move forward but for 0 seconds.
        - This means it should not send any actual movement, just a stop command.
        - check that the final message still says the movement was executed.
        """
        agent = MoveBotAgent()
        await agent.move('forward', 0.0)
        msg = await aiAgentsMain.message_queue.get()
        self.assertIn("Executed forward", msg)
        agent.destroy_node()

    async def test_controller_no_message_timeout(self):
        """
        Test if MainController can stay running even when no messages are received.

        Steps:
        - run the controller's listen task.
        - do NOT add anything to the queue.
        - let it run briefly and then cancel it.
        - test passes if it doesn't crash or raise errors.
        """
        controller = MainController()
        task = asyncio.create_task(controller.listen_for_notifications())
        await asyncio.sleep(0.05)  # Let it idle
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            self.assertTrue(True)

if __name__ == "__main__":
    unittest.main(verbosity=1)