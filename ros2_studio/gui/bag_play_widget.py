"""Widget for ROS2 bag playback with file selection."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
    QLabel, QFileDialog, QLineEdit, QGroupBox,
    QTextEdit, QSlider, QCheckBox, QDoubleSpinBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from ros2_studio.core.bag_player import BagPlayer
import os


class BagPlayWidget(QWidget):
    """Widget for playing ROS2 bags."""
    
    def __init__(self):
        """Initialize bag play widget."""
        super().__init__()
        self.bag_player = BagPlayer()
        self.is_paused = False
        self.setup_ui()
        
        # Timer for updating playback status
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_playback_status)
    
    def setup_ui(self):
        """Set up the user interface."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # File selection group
        file_group = QGroupBox("Select Bag File")
        file_layout = QVBoxLayout()
        
        # Bag file input
        file_input_layout = QHBoxLayout()
        file_label = QLabel('Bag Path:')
        file_label.setStyleSheet("font-weight: bold;")
        
        self.bag_path_input = QLineEdit()
        self.bag_path_input.setPlaceholderText('/path/to/bag/file')
        self.bag_path_input.textChanged.connect(self.on_bag_path_changed)
        
        browse_button = QPushButton('📁 Browse')
        browse_button.clicked.connect(self.browse_bag_file)
        
        file_input_layout.addWidget(file_label)
        file_input_layout.addWidget(self.bag_path_input)
        file_input_layout.addWidget(browse_button)
        
        file_layout.addLayout(file_input_layout)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Bag info group
        info_group = QGroupBox("Bag Information")
        info_layout = QVBoxLayout()
        
        self.bag_info_text = QTextEdit()
        self.bag_info_text.setReadOnly(True)
        self.bag_info_text.setMaximumHeight(120)
        self.bag_info_text.setStyleSheet("""
            QTextEdit {
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                padding: 5px;
                font-size: 11px;
                background-color: #ecf0f1;
                font-family: monospace;
            }
        """)
        self.bag_info_text.setText('Select a bag file to view information...')
        
        load_info_button = QPushButton('📊 Load Bag Info')
        load_info_button.clicked.connect(self.load_bag_info)
        
        info_layout.addWidget(self.bag_info_text)
        info_layout.addWidget(load_info_button)
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        # Playback options group
        options_group = QGroupBox("Playback Options")
        options_layout = QVBoxLayout()
        
        # Rate control
        rate_layout = QHBoxLayout()
        rate_label = QLabel('Playback Rate:')
        rate_label.setStyleSheet("font-weight: bold;")
        
        self.rate_spinbox = QDoubleSpinBox()
        self.rate_spinbox.setRange(0.1, 10.0)
        self.rate_spinbox.setValue(1.0)
        self.rate_spinbox.setSingleStep(0.1)
        self.rate_spinbox.setDecimals(1)
        self.rate_spinbox.setSuffix('x')
        
        rate_layout.addWidget(rate_label)
        rate_layout.addWidget(self.rate_spinbox)
        rate_layout.addStretch()
        
        # Loop option
        self.loop_checkbox = QCheckBox('Loop Playback')
        self.loop_checkbox.setStyleSheet("font-weight: bold;")
        
        options_layout.addLayout(rate_layout)
        options_layout.addWidget(self.loop_checkbox)
        options_group.setLayout(options_layout)
        layout.addWidget(options_group)
        
        # Playback controls group
        controls_group = QGroupBox("Playback Controls")
        controls_layout = QVBoxLayout()
        
        button_layout = QHBoxLayout()
        
        self.play_button = QPushButton('▶️ Play')
        self.pause_button = QPushButton('⏸️ Pause')
        self.resume_button = QPushButton('▶️ Resume')
        self.stop_button = QPushButton('⏹️ Stop')
        
        # Style buttons
        button_style = """
            QPushButton {
                font-weight: bold;
                font-size: 13px;
                padding: 10px 20px;
                border-radius: 5px;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
                color: white;
            }
        """
        
        self.play_button.setStyleSheet(button_style + """
            QPushButton {
                background-color: #27ae60;
                color: white;
            }
            QPushButton:hover {
                background-color: #229954;
            }
        """)
        
        self.pause_button.setStyleSheet(button_style + """
            QPushButton {
                background-color: #f39c12;
                color: white;
            }
            QPushButton:hover {
                background-color: #e67e22;
            }
        """)
        
        self.resume_button.setStyleSheet(button_style + """
            QPushButton {
                background-color: #3498db;
                color: white;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        
        self.stop_button.setStyleSheet(button_style + """
            QPushButton {
                background-color: #e74c3c;
                color: white;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        
        # Connect buttons
        self.play_button.clicked.connect(self.play_bag)
        self.pause_button.clicked.connect(self.pause_playback)
        self.resume_button.clicked.connect(self.resume_playback)
        self.stop_button.clicked.connect(self.stop_playback)
        
        # Initial button states
        self.pause_button.setEnabled(False)
        self.resume_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        
        button_layout.addWidget(self.play_button)
        button_layout.addWidget(self.pause_button)
        button_layout.addWidget(self.resume_button)
        button_layout.addWidget(self.stop_button)
        
        controls_layout.addLayout(button_layout)
        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)
        
        # Status display
        status_group = QGroupBox("Playback Status")
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel('Status: Ready')
        self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2c3e50;")
        
        self.status_info = QTextEdit()
        self.status_info.setReadOnly(True)
        self.status_info.setMaximumHeight(80)
        self.status_info.setStyleSheet("""
            QTextEdit {
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                padding: 5px;
                font-size: 11px;
                background-color: #ecf0f1;
            }
        """)
        self.status_info.setText('Ready to play. Select a bag file and click "Play".')
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.status_info)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
    
    def browse_bag_file(self):
        """Open file dialog to select bag file."""
        # Try to browse for directory (ROS2 bags are typically directories)
        default_path = self.bag_path_input.text() or os.path.expanduser('~/ros2_bags')
        
        directory = QFileDialog.getExistingDirectory(
            self,
            'Select Bag Directory',
            default_path
        )
        
        if directory:
            self.bag_path_input.setText(directory)
        else:
            # If no directory selected, try file dialog as fallback
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                'Select Bag File',
                default_path,
                'Bag Files (*.db3 *.bag);;All Files (*)'
            )
            if file_path:
                self.bag_path_input.setText(file_path)
    
    def on_bag_path_changed(self):
        """Handle bag path text change."""
        # Could add real-time validation here
        pass
    
    def load_bag_info(self):
        """Load and display bag file information."""
        bag_path = self.bag_path_input.text()
        
        if not bag_path:
            self.bag_info_text.setText('⚠ Please specify a bag file path.')
            return
        
        if not os.path.exists(bag_path):
            self.bag_info_text.setText(f'⚠ Path does not exist: {bag_path}')
            return
        
        self.bag_info_text.setText('Loading bag information...')
        
        try:
            info = self.bag_player.get_bag_info(bag_path)
            
            if info:
                info_text = f"📦 Bag Information:\n"
                info_text += f"  Path: {info['path']}\n"
                info_text += f"  Duration: {info['duration']}\n"
                info_text += f"  Size: {info['size']}\n"
                info_text += f"  Messages: {info['messages']}\n"
                
                if info['topics']:
                    info_text += f"\n📋 Topics ({len(info['topics'])}):\n"
                    for topic in info['topics'][:10]:  # Show first 10
                        info_text += f"  • {topic}\n"
                    if len(info['topics']) > 10:
                        info_text += f"  ... and {len(info['topics']) - 10} more\n"
                
                self.bag_info_text.setText(info_text)
            else:
                self.bag_info_text.setText('✗ Failed to load bag information. Check console for errors.')
        except Exception as e:
            self.bag_info_text.setText(f'✗ Error: {e}')
    
    def play_bag(self):
        """Start bag playback."""
        bag_path = self.bag_path_input.text()
        
        if not bag_path:
            self.status_label.setText('Status: ⚠ Please specify a bag file!')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        if not os.path.exists(bag_path):
            self.status_label.setText('Status: ⚠ Bag file not found!')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        rate = self.rate_spinbox.value()
        loop = self.loop_checkbox.isChecked()
        
        success = self.bag_player.play_bag(bag_path, rate=rate, loop=loop)
        
        if success:
            self.status_label.setText('Status: ▶️ Playing...')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #27ae60;")
            
            self.status_info.setText(
                f'▶ Playback started\n'
                f'  File: {os.path.basename(bag_path)}\n'
                f'  Rate: {rate}x\n'
                f'  Loop: {"Yes" if loop else "No"}'
            )
            
            self.play_button.setEnabled(False)
            self.pause_button.setEnabled(True)
            self.resume_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.is_paused = False
            
            # Start status update timer
            self.status_timer.start(1000)
        else:
            self.status_label.setText('Status: ✗ Failed to start playback!')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            self.status_info.append('\n✗ Failed to start playback. Check console for errors.')
    
    def pause_playback(self):
        """Pause current playback."""
        success = self.bag_player.pause_playback()
        
        if success:
            self.status_label.setText('Status: ⏸️ Paused')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #f39c12;")
            self.pause_button.setEnabled(False)
            self.resume_button.setEnabled(True)
            self.is_paused = True
    
    def resume_playback(self):
        """Resume paused playback."""
        success = self.bag_player.resume_playback()
        
        if success:
            self.status_label.setText('Status: ▶️ Playing...')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #27ae60;")
            self.pause_button.setEnabled(True)
            self.resume_button.setEnabled(False)
            self.is_paused = False
    
    def stop_playback(self):
        """Stop current playback."""
        self.status_timer.stop()
        success = self.bag_player.stop_playback()
        
        if success:
            self.status_label.setText('Status: ⏹️ Stopped')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2980b9;")
            self.status_info.append('\n⏹ Playback stopped')
        
        self.play_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.resume_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.is_paused = False
    
    def update_playback_status(self):
        """Update playback status periodically."""
        status = self.bag_player.get_playback_status()
        
        # Check if playback has ended
        if not status['is_playing'] and not self.is_paused:
            self.stop_playback()
            self.status_label.setText('Status: ✓ Playback completed')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2980b9;")
            self.status_info.append('\n✓ Playback completed')
    
    def cleanup(self):
        """Clean up resources."""
        try:
            self.status_timer.stop()
            self.bag_player.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
