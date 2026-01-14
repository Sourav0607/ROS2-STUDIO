"""Main window for ROS2 Studio with feature selection."""
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QComboBox, QStackedWidget, QLabel, QStatusBar
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPixmap, QIcon
import os
from ros2_studio.gui.monitoring_widget import MonitoringWidget
from ros2_studio.gui.bag_record_widget import BagRecordWidget
from ros2_studio.gui.bag_play_widget import BagPlayWidget
from ros2_studio.gui.bag_convert_widget import BagConvertWidget
from ros2_studio.gui.system_dashboard_widget import SystemDashboardWidget


class MainWindow(QMainWindow):
    """Main application window with feature selection."""
    
    def __init__(self):
        """Initialize the main window."""
        super().__init__()
        self.setWindowTitle('ROS2 Studio - Monitoring & Management Tool')
        self.setGeometry(100, 100, 1400, 900)
        
        # Set window icon
        try:
            icon_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                '3d-cute-robot-working-touching-260nw-2698930129.jpg'
            )
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
        except Exception as e:
            print(f"Could not load icon: {e}")
        
        # Set up central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # Title and feature selection
        self._setup_header(main_layout)
        
        # Stacked widget for different features
        self.stacked_widget = QStackedWidget()
        
        # Initialize feature widgets
        try:
            self.monitoring_widget = MonitoringWidget()
            self.bag_record_widget = BagRecordWidget()
            self.bag_play_widget = BagPlayWidget()
            self.bag_convert_widget = BagConvertWidget()
            self.dashboard_widget = SystemDashboardWidget()
            
            self.stacked_widget.addWidget(self.monitoring_widget)
            self.stacked_widget.addWidget(self.bag_record_widget)
            self.stacked_widget.addWidget(self.bag_play_widget)
            self.stacked_widget.addWidget(self.bag_convert_widget)
            self.stacked_widget.addWidget(self.dashboard_widget)
        except Exception as e:
            print(f"Error initializing widgets: {e}")
            # Create placeholder widgets if initialization fails
            self.stacked_widget.addWidget(QLabel("Error loading monitoring widget"))
            self.stacked_widget.addWidget(QLabel("Error loading bag record widget"))
            self.stacked_widget.addWidget(QLabel("Error loading bag play widget"))
            self.stacked_widget.addWidget(QLabel("Error loading bag convert widget"))
            self.stacked_widget.addWidget(QLabel("Error loading dashboard widget"))
        
        main_layout.addWidget(self.stacked_widget)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage('Ready')
    
    def _setup_header(self, layout):
        """Set up the header with title and feature selector."""
        # Header layout
        header_layout = QVBoxLayout()
        
        # Title
        title_label = QLabel('🤖 ROS2 Studio')
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        
        # Feature selection
        feature_layout = QHBoxLayout()
        feature_label = QLabel('Select Feature:')
        feature_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        
        self.feature_selector = QComboBox()
        self.feature_selector.addItems([
            '📊 Performance Monitor',
            '🔴 Bag Recorder',
            '▶️ Bag Player',
            '🔄 Bag to CSV Converter',
            '🎛️ System Dashboard'
        ])
        self.feature_selector.setStyleSheet("""
            QComboBox {
                font-size: 13px;
                padding: 8px;
                border: 2px solid #3498db;
                border-radius: 5px;
                background-color: white;
                color: black;
                min-width: 250px;
            }
            QComboBox:hover {
                border: 2px solid #2980b9;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #3498db;
            }
            QComboBox QAbstractItemView {
                background-color: white;
                color: black;
                selection-background-color: #3498db;
                selection-color: white;
                border: 1px solid #3498db;
            }
            QComboBox QAbstractItemView::item {
                padding: 8px;
                color: black;
            }
            QComboBox QAbstractItemView::item:hover {
                background-color: #5dade2;
                color: white;
            }
        """)
        self.feature_selector.currentIndexChanged.connect(self.change_feature)
        
        feature_layout.addStretch()
        feature_layout.addWidget(feature_label)
        feature_layout.addWidget(self.feature_selector)
        feature_layout.addStretch()
        
        # Add to header
        header_layout.addWidget(title_label)
        header_layout.addSpacing(10)
        header_layout.addLayout(feature_layout)
        
        layout.addLayout(header_layout)
    
    def change_feature(self, index):
        """Change the displayed feature widget."""
        self.stacked_widget.setCurrentIndex(index)
        
        # Update status bar
        features = ['Performance Monitor', 'Bag Recorder', 'Bag Player', 'Bag to CSV Converter']
        if 0 <= index < len(features):
            self.status_bar.showMessage(f'Active: {features[index]}')
    
    def closeEvent(self, event):
        """Handle window close event."""
        # Clean up widgets
        try:
            if hasattr(self, 'monitoring_widget'):
                self.monitoring_widget.cleanup()
            if hasattr(self, 'bag_record_widget'):
                self.bag_record_widget.cleanup()
            if hasattr(self, 'bag_play_widget'):
                self.bag_play_widget.cleanup()
            if hasattr(self, 'bag_convert_widget'):
                self.bag_convert_widget.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
        
        event.accept()
