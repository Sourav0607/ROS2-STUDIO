"""Backend for ROS2 system dashboard and diagnostics."""
import rclpy
from rclpy.node import Node
import psutil
import threading
import time
from collections import defaultdict


class SystemDashboard:
    """Monitor ROS2 system health and statistics."""
    
    def __init__(self):
        """Initialize system dashboard."""
        self.node = None
        self.executor = None
        self.executor_thread = None
        
        self._initialize_node()
    
    def _initialize_node(self):
        """Initialize ROS2 node and executor."""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('ros2_studio_dashboard')
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Start executor in separate thread
            self.executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True
            )
            self.executor_thread.start()
        except Exception as e:
            print(f"Error initializing dashboard: {e}")
    
    def get_system_overview(self):
        """Get overall system statistics."""
        if not self.node:
            return None
        
        try:
            # System resources
            cpu_percent = psutil.cpu_percent(interval=0.1, percpu=True)
            cpu_avg = sum(cpu_percent) / len(cpu_percent)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            
            # Network stats
            net_io = psutil.net_io_counters()
            
            # ROS2 entities
            topics = self.node.get_topic_names_and_types()
            nodes = self.node.get_node_names()
            
            # Count topics by namespace
            namespaces = defaultdict(int)
            for topic, _ in topics:
                namespace = '/'.join(topic.split('/')[:-1]) or '/'
                namespaces[namespace] += 1
            
            return {
                'cpu': {
                    'average': cpu_avg,
                    'per_core': cpu_percent,
                    'count': psutil.cpu_count()
                },
                'memory': {
                    'total': memory.total / (1024**3),  # GB
                    'used': memory.used / (1024**3),
                    'percent': memory.percent
                },
                'disk': {
                    'total': disk.total / (1024**3),
                    'used': disk.used / (1024**3),
                    'percent': disk.percent
                },
                'network': {
                    'bytes_sent': net_io.bytes_sent / (1024**2),  # MB
                    'bytes_recv': net_io.bytes_recv / (1024**2),
                    'packets_sent': net_io.packets_sent,
                    'packets_recv': net_io.packets_recv
                },
                'ros2': {
                    'nodes_count': len(nodes),
                    'topics_count': len(topics),
                    'namespaces': dict(namespaces)
                }
            }
        except Exception as e:
            print(f"Error getting system overview: {e}")
            return None
    
    def get_topic_statistics(self):
        """Get detailed topic statistics."""
        if not self.node:
            return []
        
        try:
            topics = self.node.get_topic_names_and_types()
            topic_stats = []
            
            for topic_name, topic_types in topics:
                pub_count = self.node.count_publishers(topic_name)
                sub_count = self.node.count_subscribers(topic_name)
                
                topic_stats.append({
                    'name': topic_name,
                    'type': topic_types[0] if topic_types else 'unknown',
                    'publishers': pub_count,
                    'subscribers': sub_count
                })
            
            return sorted(topic_stats, key=lambda x: x['name'])
        except Exception as e:
            print(f"Error getting topic statistics: {e}")
            return []
    
    def get_node_details(self):
        """Get detailed node information."""
        if not self.node:
            return []
        
        try:
            nodes = self.node.get_node_names()
            node_details = []
            
            for node_name in nodes:
                # Get node namespace
                namespace = self.node.get_node_names_and_namespaces()
                node_ns = next((ns for name, ns in namespace if name == node_name), '/')
                
                node_details.append({
                    'name': node_name,
                    'namespace': node_ns,
                    'full_name': f"{node_ns}/{node_name}" if node_ns != '/' else f"/{node_name}"
                })
            
            return sorted(node_details, key=lambda x: x['name'])
        except Exception as e:
            print(f"Error getting node details: {e}")
            return []
    
    def get_process_info(self):
        """Get information about running processes."""
        try:
            ros_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
                try:
                    pinfo = proc.info
                    # Filter ROS-related processes
                    if any(keyword in pinfo['name'].lower() for keyword in 
                          ['ros', 'gazebo', 'rviz', 'moveit', 'navigation']):
                        ros_processes.append({
                            'pid': pinfo['pid'],
                            'name': pinfo['name'],
                            'cpu': pinfo['cpu_percent'] or 0,
                            'memory': pinfo['memory_percent'] or 0
                        })
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            
            return sorted(ros_processes, key=lambda x: x['cpu'], reverse=True)[:20]
        except Exception as e:
            print(f"Error getting process info: {e}")
            return []
    
    def get_network_interfaces(self):
        """Get network interface statistics."""
        try:
            interfaces = {}
            net_if_stats = psutil.net_if_stats()
            net_if_addrs = psutil.net_if_addrs()
            
            for interface, stats in net_if_stats.items():
                addrs = net_if_addrs.get(interface, [])
                ip_addr = next((addr.address for addr in addrs 
                              if addr.family == 2), 'N/A')  # AF_INET = 2
                
                interfaces[interface] = {
                    'is_up': stats.isup,
                    'speed': stats.speed,  # Mbps
                    'ip_address': ip_addr
                }
            
            return interfaces
        except Exception as e:
            print(f"Error getting network interfaces: {e}")
            return {}
    
    def cleanup(self):
        """Clean up resources."""
        try:
            if self.executor:
                self.executor.shutdown()
            if self.node:
                self.node.destroy_node()
        except Exception as e:
            print(f"Error during cleanup: {e}")
