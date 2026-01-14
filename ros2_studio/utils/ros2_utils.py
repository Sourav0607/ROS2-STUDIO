"""ROS2 utilities for topic and node discovery."""
import rclpy
from rclpy.node import Node
import threading
import time


class ROS2Utils:
    """Utility class for ROS2 operations."""
    
    def __init__(self):
        """Initialize ROS2 utilities."""
        self.node = None
        self.executor = None
        self.executor_thread = None
        self._initialize_node()
    
    def _initialize_node(self):
        """Initialize ROS2 node and executor."""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('ros2_studio_utils')
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Start executor in separate thread
            self.executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True
            )
            self.executor_thread.start()
        except Exception as e:
            print(f"Error initializing ROS2 node: {e}")
    
    def get_all_topics(self):
        """Get list of all active topics."""
        if not self.node:
            return []
        
        try:
            topic_list = self.node.get_topic_names_and_types()
            return sorted([topic[0] for topic in topic_list])
        except Exception as e:
            print(f"Error getting topics: {e}")
            return []
    
    def get_all_nodes(self):
        """Get list of all active nodes."""
        if not self.node:
            return []
        
        try:
            node_names = self.node.get_node_names()
            return sorted(node_names)
        except Exception as e:
            print(f"Error getting nodes: {e}")
            return []
    
    def get_topic_info(self, topic_name):
        """Get information about a specific topic."""
        if not self.node:
            return None
        
        try:
            topic_list = self.node.get_topic_names_and_types()
            for topic, types in topic_list:
                if topic == topic_name:
                    return {
                        'name': topic,
                        'types': types,
                    }
            return None
        except Exception as e:
            print(f"Error getting topic info: {e}")
            return None
    
    def get_publishers_info(self, topic_name):
        """Get publisher count for a topic."""
        if not self.node:
            return 0
        
        try:
            publishers = self.node.count_publishers(topic_name)
            return publishers
        except Exception as e:
            print(f"Error getting publisher info: {e}")
            return 0
    
    def get_subscribers_info(self, topic_name):
        """Get subscriber count for a topic."""
        if not self.node:
            return 0
        
        try:
            subscribers = self.node.count_subscribers(topic_name)
            return subscribers
        except Exception as e:
            print(f"Error getting subscriber info: {e}")
            return 0
    
    def cleanup(self):
        """Clean up resources."""
        try:
            if self.executor:
                self.executor.shutdown()
            if self.node:
                self.node.destroy_node()
        except Exception as e:
            print(f"Error during cleanup: {e}")
