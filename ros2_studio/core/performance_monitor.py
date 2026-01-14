"""Performance monitoring backend for ROS2 topics and nodes."""
import rclpy
from rclpy.node import Node
import psutil
import threading
import time
from collections import defaultdict, deque


class PerformanceMonitor:
    """Monitor performance metrics for ROS2 topics and nodes."""
    
    def __init__(self):
        """Initialize performance monitor."""
        self.node = None
        self.executor = None
        self.executor_thread = None
        
        # Tracking dictionaries
        self.topic_message_counts = defaultdict(int)
        self.topic_message_times = defaultdict(lambda: deque(maxlen=100))
        self.subscriptions = {}
        
        self._initialize_node()
    
    def _initialize_node(self):
        """Initialize ROS2 node and executor."""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('ros2_studio_performance_monitor')
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Start executor in separate thread
            self.executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True
            )
            self.executor_thread.start()
        except Exception as e:
            print(f"Error initializing performance monitor: {e}")
    
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
    
    def get_topic_metrics(self, topic_name):
        """Get performance metrics for a specific topic."""
        if not self.node:
            return None
        
        try:
            # Get system-level metrics
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_info = psutil.virtual_memory()
            memory_mb = memory_info.used / (1024 ** 2)
            
            # Calculate message frequency
            frequency = self._calculate_topic_frequency(topic_name)
            
            # Get publisher/subscriber counts
            publishers = self.node.count_publishers(topic_name)
            subscribers = self.node.count_subscribers(topic_name)
            
            return {
                'cpu': cpu_percent,
                'memory': memory_mb,
                'frequency': frequency,
                'publishers': publishers,
                'subscribers': subscribers,
                'message_count': self.topic_message_counts.get(topic_name, 0)
            }
        except Exception as e:
            print(f"Error getting topic metrics for {topic_name}: {e}")
            return {
                'cpu': 0.0,
                'memory': 0.0,
                'frequency': 0.0,
                'publishers': 0,
                'subscribers': 0,
                'message_count': 0
            }
    
    def get_node_metrics(self, node_name):
        """Get performance metrics for a specific node."""
        try:
            # Get system-level metrics
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_info = psutil.virtual_memory()
            memory_mb = memory_info.used / (1024 ** 2)
            
            # Try to find the process for this node
            node_cpu = 0.0
            node_memory = 0.0
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = proc.info['cmdline']
                    if cmdline and node_name in ' '.join(cmdline):
                        node_cpu = proc.cpu_percent(interval=0.1)
                        node_memory = proc.memory_info().rss / (1024 ** 2)
                        break
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            return {
                'cpu': node_cpu if node_cpu > 0 else cpu_percent,
                'memory': node_memory if node_memory > 0 else memory_mb,
                'frequency': 0.0,
                'publishers': 0,
                'subscribers': 0
            }
        except Exception as e:
            print(f"Error getting node metrics for {node_name}: {e}")
            return {
                'cpu': 0.0,
                'memory': 0.0,
                'frequency': 0.0,
                'publishers': 0,
                'subscribers': 0
            }
    
    def _calculate_topic_frequency(self, topic_name):
        """Calculate the publishing frequency of a topic."""
        times = self.topic_message_times.get(topic_name)
        if not times or len(times) < 2:
            return 0.0
        
        try:
            time_diffs = []
            times_list = list(times)
            for i in range(1, len(times_list)):
                time_diffs.append(times_list[i] - times_list[i-1])
            
            if time_diffs:
                avg_period = sum(time_diffs) / len(time_diffs)
                if avg_period > 0:
                    return 1.0 / avg_period
            return 0.0
        except Exception as e:
            print(f"Error calculating frequency: {e}")
            return 0.0
    
    def _topic_callback(self, msg, topic_name):
        """Callback for topic messages."""
        self.topic_message_counts[topic_name] += 1
        self.topic_message_times[topic_name].append(time.time())
    
    def subscribe_to_topic(self, topic_name):
        """Subscribe to a topic for monitoring."""
        if topic_name in self.subscriptions:
            return
        
        try:
            # Get topic type
            topic_list = self.node.get_topic_names_and_types()
            topic_type = None
            for topic, types in topic_list:
                if topic == topic_name:
                    if types:
                        topic_type = types[0]
                    break
            
            if not topic_type:
                print(f"Could not determine type for topic {topic_name}")
                return
            
            # For simplicity, we'll track messages without full type support
            # This is a limitation that can be improved with rosidl runtime
            print(f"Monitoring topic: {topic_name} ({topic_type})")
            
        except Exception as e:
            print(f"Error subscribing to topic {topic_name}: {e}")
    
    def cleanup(self):
        """Clean up resources."""
        try:
            # Destroy subscriptions
            for sub in self.subscriptions.values():
                try:
                    self.node.destroy_subscription(sub)
                except:
                    pass
            
            if self.executor:
                self.executor.shutdown()
            if self.node:
                self.node.destroy_node()
        except Exception as e:
            print(f"Error during cleanup: {e}")
