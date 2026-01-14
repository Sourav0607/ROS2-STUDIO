"""Backend for ROS2 bag to CSV conversion functionality."""
import subprocess
import os
from datetime import datetime
import csv
import yaml
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    ROSBAG2_AVAILABLE = True
except ImportError:
    ROSBAG2_AVAILABLE = False
    print("Warning: rosbag2_py not available. CSV conversion will be limited.")


class BagConverter:
    """Handle ROS2 bag to CSV conversion operations."""
    
    def __init__(self):
        """Initialize bag converter."""
        self.conversion_process = None
        self.is_converting = False
    
    def get_bag_info(self, bag_path):
        """
        Get information about a bag file.
        
        Args:
            bag_path: Path to the bag file/directory
            
        Returns:
            Dictionary with bag information including topics
        """
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', bag_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                output = result.stdout
                print(f"Bag info output:\n{output}")  # Debug output
                
                info = {
                    'path': bag_path,
                    'topics': [],
                    'topic_types': {},
                    'duration': 'Unknown',
                    'messages': 'Unknown',
                    'size': 'Unknown'
                }
                
                lines = output.split('\n')
                parsing_topics = False
                
                for line in lines:
                    original_line = line
                    line = line.strip()
                    
                    if 'Duration:' in line:
                        info['duration'] = line.split(':', 1)[1].strip()
                    elif 'Bag size:' in line:
                        info['size'] = line.split(':', 1)[1].strip()
                    elif 'Messages:' in line and 'Topic' not in line:
                        info['messages'] = line.split(':', 1)[1].strip()
                    elif 'Topic information:' in line:
                        # The topic info might be on the same line or following lines
                        parsing_topics = True
                        # Check if topic info is on the same line
                        if 'Topic:' in line:
                            # Extract the part after "Topic information:"
                            topic_part = line.split('Topic information:', 1)[1].strip()
                            if topic_part and '|' in topic_part:
                                parts = topic_part.split('|')
                                topic_name = None
                                topic_type = 'Unknown'
                                
                                for part in parts:
                                    part = part.strip()
                                    if part.startswith('Topic:'):
                                        topic_name = part.replace('Topic:', '').strip()
                                    elif part.startswith('Type:'):
                                        topic_type = part.replace('Type:', '').strip()
                                
                                if topic_name:
                                    info['topics'].append(topic_name)
                                    info['topic_types'][topic_name] = topic_type
                    elif parsing_topics and line and 'Topic:' in line and '|' in line:
                        # Process topic lines that appear after "Topic information:"
                        parts = line.split('|')
                        topic_name = None
                        topic_type = 'Unknown'
                        
                        for part in parts:
                            part = part.strip()
                            if part.startswith('Topic:'):
                                topic_name = part.replace('Topic:', '').strip()
                            elif part.startswith('Type:'):
                                topic_type = part.replace('Type:', '').strip()
                        
                        if topic_name:
                            info['topics'].append(topic_name)
                            info['topic_types'][topic_name] = topic_type
                
                print(f"Parsed topics: {info['topics']}")  # Debug output
                return info
            else:
                print(f"Error getting bag info: {result.stderr}")
                return None
                
        except Exception as e:
            print(f"Error getting bag info: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def convert_bag_to_csv(self, bag_path, output_dir, selected_topics=None):
        """
        Convert bag file to CSV format.
        
        Args:
            bag_path: Path to the bag file/directory
            output_dir: Directory where CSV files will be saved
            selected_topics: List of topics to convert (None = all topics)
            
        Returns:
            Dictionary with conversion results
        """
        if self.is_converting:
            return {'success': False, 'error': 'Already converting a bag'}
        
        if not os.path.exists(bag_path):
            return {'success': False, 'error': f'Bag file not found: {bag_path}'}
        
        try:
            # Create output directory
            os.makedirs(output_dir, exist_ok=True)
            
            # Get bag info
            bag_info = self.get_bag_info(bag_path)
            if not bag_info:
                return {'success': False, 'error': 'Failed to read bag information'}
            
            # Determine topics to convert
            topics_to_convert = selected_topics if selected_topics else bag_info['topics']
            
            if not topics_to_convert:
                return {'success': False, 'error': 'No topics to convert'}
            
            self.is_converting = True
            converted_files = []
            errors = []
            
            # Convert each topic
            for topic in topics_to_convert:
                try:
                    # Generate CSV filename
                    topic_name_safe = topic.replace('/', '_').strip('_')
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    csv_filename = f"{topic_name_safe}_{timestamp}.csv"
                    csv_path = os.path.join(output_dir, csv_filename)
                    
                    # Use ros2 bag info to get message type
                    topic_type = bag_info['topic_types'].get(topic, 'Unknown')
                    
                    # Convert using ros2 bag play and ros2 topic echo
                    # This is a simplified approach - for production, you'd want to use rosbag2_py
                    result = self._convert_topic_to_csv(bag_path, topic, csv_path)
                    
                    if result:
                        converted_files.append({
                            'topic': topic,
                            'file': csv_path,
                            'type': topic_type
                        })
                    else:
                        errors.append(f"Failed to convert {topic}")
                        
                except Exception as e:
                    errors.append(f"Error converting {topic}: {str(e)}")
            
            self.is_converting = False
            
            return {
                'success': len(converted_files) > 0,
                'converted_files': converted_files,
                'errors': errors,
                'total': len(topics_to_convert),
                'successful': len(converted_files)
            }
            
        except Exception as e:
            self.is_converting = False
            return {'success': False, 'error': str(e)}
    
    def _convert_topic_to_csv(self, bag_path, topic, csv_path):
        """
        Convert a single topic to CSV using rosbag2_py.
        """
        if not ROSBAG2_AVAILABLE:
            print("rosbag2_py not available, using fallback method")
            return self._convert_topic_fallback(bag_path, topic, csv_path)
        
        try:
            # Create reader
            storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
            converter_options = rosbag2_py.ConverterOptions('', '')
            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)
            
            # Get topic metadata
            topic_types = reader.get_all_topics_and_types()
            type_map = {t.name: t.type for t in topic_types}
            
            if topic not in type_map:
                print(f"Topic {topic} not found in bag")
                return False
            
            # Get message type
            msg_type = type_map[topic]
            
            try:
                msg_class = get_message(msg_type)
            except Exception as e:
                print(f"Could not load message type {msg_type}: {e}")
                return self._convert_topic_fallback(bag_path, topic, csv_path)
            
            # Set filter to read only this topic
            storage_filter = rosbag2_py.StorageFilter(topics=[topic])
            reader.set_filter(storage_filter)
            
            # Read messages and write to CSV
            messages_data = []
            
            while reader.has_next():
                (topic_name, data, timestamp) = reader.read_next()
                
                if topic_name == topic:
                    try:
                        # Deserialize message
                        msg = deserialize_message(data, msg_class)
                        
                        # Convert timestamp to datetime
                        dt = datetime.fromtimestamp(timestamp / 1e9)
                        
                        # Flatten message to dict
                        msg_dict = self._message_to_dict(msg)
                        msg_dict['timestamp'] = dt.isoformat()
                        msg_dict['ros_timestamp_ns'] = timestamp
                        
                        messages_data.append(msg_dict)
                        
                    except Exception as e:
                        print(f"Error deserializing message: {e}")
                        continue
            
            # Write to CSV
            if messages_data:
                with open(csv_path, 'w', newline='') as csvfile:
                    # Get all field names from all messages
                    fieldnames = set()
                    for msg in messages_data:
                        fieldnames.update(msg.keys())
                    fieldnames = sorted(fieldnames)
                    
                    # Move timestamp to first column
                    if 'timestamp' in fieldnames:
                        fieldnames.remove('timestamp')
                        fieldnames.insert(0, 'timestamp')
                    
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(messages_data)
                
                print(f"Successfully converted {len(messages_data)} messages from {topic}")
                return True
            else:
                print(f"No messages found for topic {topic}")
                return False
            
        except Exception as e:
            print(f"Error converting topic {topic}: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _message_to_dict(self, msg, parent_key=''):
        """
        Convert a ROS message to a flat dictionary.
        """
        result = {}
        
        # Get all fields from the message
        if hasattr(msg, '__slots__'):
            for slot in msg.__slots__:
                value = getattr(msg, slot)
                key = f"{parent_key}.{slot}" if parent_key else slot
                
                # Handle different types
                if hasattr(value, '__slots__'):
                    # Nested message
                    nested = self._message_to_dict(value, key)
                    result.update(nested)
                elif isinstance(value, (list, tuple)):
                    # Array - convert to string or handle primitives
                    if value and hasattr(value[0], '__slots__'):
                        # Array of messages - too complex, store as string
                        result[key] = str(value)
                    else:
                        # Array of primitives
                        result[key] = str(value)
                elif isinstance(value, bytes):
                    # Byte array
                    result[key] = value.hex()
                else:
                    # Primitive type
                    result[key] = value
        else:
            # Simple type
            result[parent_key] = str(msg)
        
        return result
    
    def _convert_topic_fallback(self, bag_path, topic, csv_path):
        """
        Fallback method when rosbag2_py is not available.
        Creates a basic CSV with message info.
        """
        try:
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'topic', 'note'])
                writer.writerow([
                    datetime.now().isoformat(), 
                    topic, 
                    'Full conversion requires rosbag2_py. Install with: sudo apt install ros-$ROS_DISTRO-rosbag2-py'
                ])
            return True
        except Exception as e:
            print(f"Error in fallback conversion: {e}")
            return False
    
    def get_conversion_status(self):
        """
        Get current conversion status.
        
        Returns:
            Dictionary with conversion status
        """
        return {
            'is_converting': self.is_converting
        }
    
    def cleanup(self):
        """Clean up resources."""
        self.is_converting = False
