"""Main entry point for ROS2 Studio GUI application."""
import sys
import rclpy
from PyQt5.QtWidgets import QApplication
from ros2_studio.gui.main_window import MainWindow


def main():
    """Initialize and run the ROS2 Studio GUI application."""
    # Initialize ROS2
    rclpy.init()
    
    # Create Qt application
    app = QApplication(sys.argv)
    app.setApplicationName('ROS2 Studio')
    app.setOrganizationName('ROS2')
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    # Run event loop
    try:
        exit_code = app.exec_()
    finally:
        # Cleanup ROS2
        rclpy.shutdown()
    
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
