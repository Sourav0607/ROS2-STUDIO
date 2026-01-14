"""Backend for ROS2 bag recording functionality."""
import subprocess
import os
from datetime import datetime
import threading
import signal


class BagRecorder:
    """Handle ROS2 bag recording operations."""
    
    def __init__(self):
        """Initialize bag recorder."""
        self.recording_process = None
        self.recording_topics = []
        self.save_location = None
        self.bag_path = None
        self.is_recording = False
    
    def get_all_topics(self):
        """Get list of all active topics."""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                topics = [topic.strip() for topic in result.stdout.strip().split('\n') if topic.strip()]
                return sorted(topics)
            return []
        except Exception as e:
            print(f"Error getting topics: {e}")
            return []
    
    def start_recording(self, topics, save_location):
        """
        Start recording selected topics to a bag file.
        
        Args:
            topics: List of topic names to record
            save_location: Directory path where bag should be saved
        """
        if self.is_recording:
            print("Already recording!")
            return False
        
        if not topics:
            print("No topics specified!")
            return False
        
        try:
            # Create save directory if it doesn't exist
            os.makedirs(save_location, exist_ok=True)
            
            # Generate bag name with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            bag_name = f'ros2_studio_bag_{timestamp}'
            self.bag_path = os.path.join(save_location, bag_name)
            
            # Build command
            cmd = ['ros2', 'bag', 'record']
            cmd.extend(topics)
            cmd.extend(['-o', self.bag_path])
            
            # Start recording process
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.recording_topics = topics
            self.save_location = save_location
            self.is_recording = True
            
            print(f"Started recording {len(topics)} topics to {self.bag_path}")
            return True
            
        except Exception as e:
            print(f"Error starting recording: {e}")
            self.is_recording = False
            return False
    
    def stop_recording(self):
        """
        Stop the current recording.
        
        Returns:
            Path to the saved bag file, or None if not recording
        """
        if not self.is_recording or not self.recording_process:
            print("Not currently recording!")
            return None
        
        try:
            # Send SIGINT to the process group to gracefully stop recording
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
            
            # Wait for process to complete
            self.recording_process.wait(timeout=10)
            
            saved_path = self.bag_path
            
            # Reset state
            self.recording_process = None
            self.is_recording = False
            self.recording_topics = []
            
            print(f"Stopped recording. Bag saved to: {saved_path}")
            return saved_path
            
        except subprocess.TimeoutExpired:
            # Force kill if graceful shutdown fails
            print("Timeout waiting for recording to stop, forcing...")
            try:
                os.killpg(os.getpgid(self.recording_process.pid), signal.SIGKILL)
                self.recording_process.wait(timeout=5)
            except:
                pass
            self.recording_process = None
            self.is_recording = False
            return self.bag_path
            
        except Exception as e:
            print(f"Error stopping recording: {e}")
            self.recording_process = None
            self.is_recording = False
            return None
    
    def get_recording_status(self):
        """
        Get current recording status.
        
        Returns:
            Dictionary with recording status information
        """
        return {
            'is_recording': self.is_recording,
            'topics': self.recording_topics,
            'save_location': self.save_location,
            'bag_path': self.bag_path
        }
    
    def cleanup(self):
        """Clean up resources."""
        if self.is_recording:
            self.stop_recording()
