"""Backend for ROS2 bag playback functionality."""
import subprocess
import os
import signal
import yaml


class BagPlayer:
    """Handle ROS2 bag playback operations."""
    
    def __init__(self):
        """Initialize bag player."""
        self.playback_process = None
        self.bag_file = None
        self.is_playing = False
    
    def get_bag_info(self, bag_path):
        """
        Get information about a bag file.
        
        Args:
            bag_path: Path to the bag file/directory
            
        Returns:
            Dictionary with bag information
        """
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', bag_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                # Parse the output
                output = result.stdout
                info = {
                    'path': bag_path,
                    'topics': [],
                    'duration': 'Unknown',
                    'messages': 'Unknown',
                    'size': 'Unknown'
                }
                
                # Simple parsing of bag info
                lines = output.split('\n')
                parsing_topics = False
                
                for line in lines:
                    line = line.strip()
                    
                    if 'Duration:' in line:
                        info['duration'] = line.split(':', 1)[1].strip()
                    elif 'Size:' in line:
                        info['size'] = line.split(':', 1)[1].strip()
                    elif 'messages' in line.lower() and ':' in line:
                        info['messages'] = line.split(':', 1)[1].strip()
                    elif 'Topic information:' in line:
                        parsing_topics = True
                    elif parsing_topics and line and '|' in line:
                        # Parse topic info (format varies)
                        parts = [p.strip() for p in line.split('|')]
                        if len(parts) >= 2 and parts[0] and not parts[0].startswith('Topic'):
                            info['topics'].append(parts[0])
                
                return info
            else:
                print(f"Error getting bag info: {result.stderr}")
                return None
                
        except subprocess.TimeoutExpired:
            print("Timeout while getting bag info")
            return None
        except Exception as e:
            print(f"Error getting bag info: {e}")
            return None
    
    def play_bag(self, bag_path, rate=1.0, loop=False):
        """
        Play a bag file.
        
        Args:
            bag_path: Path to the bag file/directory
            rate: Playback rate (1.0 = normal speed)
            loop: Whether to loop playback
            
        Returns:
            True if playback started successfully
        """
        if self.is_playing:
            print("Already playing a bag!")
            return False
        
        if not os.path.exists(bag_path):
            print(f"Bag file not found: {bag_path}")
            return False
        
        try:
            # Build command
            cmd = ['ros2', 'bag', 'play', bag_path]
            
            if rate != 1.0:
                cmd.extend(['--rate', str(rate)])
            
            if loop:
                cmd.append('--loop')
            
            # Start playback process
            self.playback_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.bag_file = bag_path
            self.is_playing = True
            
            print(f"Started playing bag: {bag_path}")
            return True
            
        except Exception as e:
            print(f"Error starting playback: {e}")
            self.is_playing = False
            return False
    
    def pause_playback(self):
        """Pause the current playback (sends SIGSTOP)."""
        if not self.is_playing or not self.playback_process:
            return False
        
        try:
            os.killpg(os.getpgid(self.playback_process.pid), signal.SIGSTOP)
            return True
        except Exception as e:
            print(f"Error pausing playback: {e}")
            return False
    
    def resume_playback(self):
        """Resume paused playback (sends SIGCONT)."""
        if not self.is_playing or not self.playback_process:
            return False
        
        try:
            os.killpg(os.getpgid(self.playback_process.pid), signal.SIGCONT)
            return True
        except Exception as e:
            print(f"Error resuming playback: {e}")
            return False
    
    def stop_playback(self):
        """
        Stop the current playback.
        
        Returns:
            True if successfully stopped
        """
        if not self.is_playing or not self.playback_process:
            print("Not currently playing!")
            return False
        
        try:
            # Send SIGINT to the process group to gracefully stop
            os.killpg(os.getpgid(self.playback_process.pid), signal.SIGINT)
            
            # Wait for process to complete
            self.playback_process.wait(timeout=5)
            
            # Reset state
            self.playback_process = None
            self.is_playing = False
            self.bag_file = None
            
            print("Stopped playback")
            return True
            
        except subprocess.TimeoutExpired:
            # Force kill if graceful shutdown fails
            print("Timeout waiting for playback to stop, forcing...")
            try:
                os.killpg(os.getpgid(self.playback_process.pid), signal.SIGKILL)
                self.playback_process.wait(timeout=3)
            except:
                pass
            self.playback_process = None
            self.is_playing = False
            self.bag_file = None
            return True
            
        except Exception as e:
            print(f"Error stopping playback: {e}")
            self.playback_process = None
            self.is_playing = False
            return False
    
    def get_playback_status(self):
        """
        Get current playback status.
        
        Returns:
            Dictionary with playback status information
        """
        # Check if process is still running
        if self.playback_process and self.is_playing:
            poll_result = self.playback_process.poll()
            if poll_result is not None:
                # Process has ended
                self.is_playing = False
                self.playback_process = None
        
        return {
            'is_playing': self.is_playing,
            'bag_file': self.bag_file
        }
    
    def cleanup(self):
        """Clean up resources."""
        if self.is_playing:
            self.stop_playback()
