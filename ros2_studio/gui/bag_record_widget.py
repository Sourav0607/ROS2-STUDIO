"""Widget for ROS2 bag recording with topic selection."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QListWidget, 
    QPushButton, QLabel, QFileDialog, QLineEdit,
    QGroupBox, QTextEdit, QAbstractItemView
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from ros2_studio.core.bag_recorder import BagRecorder
import os


class BagRecordWidget(QWidget):
    """Widget for recording ROS2 bags."""
    
    def __init__(self):
        """Initialize bag record widget."""
        super().__init__()
        self.bag_recorder = BagRecorder()
        self.recording_start_time = None
        self.setup_ui()
        
        # Timer for updating recording duration
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_recording_info)
    
    def setup_ui(self):
        """Set up the user interface."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Topic selection group
        topic_group = QGroupBox("Select Topics to Record")
        topic_layout = QVBoxLayout()
        
        # Topic list
        topic_header = QHBoxLayout()
        topic_label = QLabel('Available Topics:')
        topic_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        
        self.refresh_topics_button = QPushButton('🔄 Refresh Topics')
        self.refresh_topics_button.clicked.connect(self.refresh_topics)
        
        topic_header.addWidget(topic_label)
        topic_header.addStretch()
        topic_header.addWidget(self.refresh_topics_button)
        
        self.topic_list = QListWidget()
        self.topic_list.setSelectionMode(QAbstractItemView.MultiSelection)
        self.topic_list.setStyleSheet("""
            QListWidget {
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                padding: 5px;
                font-size: 11px;
            }
            QListWidget::item:selected {
                background-color: #3498db;
                color: white;
            }
        """)
        
        # Selection buttons
        selection_buttons = QHBoxLayout()
        self.select_all_button = QPushButton('Select All')
        self.clear_selection_button = QPushButton('Clear Selection')
        self.select_all_button.clicked.connect(self.select_all_topics)
        self.clear_selection_button.clicked.connect(self.clear_topic_selection)
        
        selection_buttons.addWidget(self.select_all_button)
        selection_buttons.addWidget(self.clear_selection_button)
        selection_buttons.addStretch()
        
        topic_layout.addLayout(topic_header)
        topic_layout.addWidget(self.topic_list)
        topic_layout.addLayout(selection_buttons)
        
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # Save location group
        location_group = QGroupBox("Save Location")
        location_layout = QHBoxLayout()
        
        location_label = QLabel('Directory:')
        location_label.setStyleSheet("font-weight: bold;")
        
        self.location_input = QLineEdit()
        self.location_input.setPlaceholderText('/path/to/save/bags')
        default_path = os.path.expanduser('~/ros2_bags')
        self.location_input.setText(default_path)
        
        browse_button = QPushButton('📁 Browse')
        browse_button.clicked.connect(self.browse_location)
        
        location_layout.addWidget(location_label)
        location_layout.addWidget(self.location_input)
        location_layout.addWidget(browse_button)
        
        location_group.setLayout(location_layout)
        layout.addWidget(location_group)
        
        # Control buttons
        control_group = QGroupBox("Recording Controls")
        control_layout = QVBoxLayout()
        
        button_layout = QHBoxLayout()
        self.start_button = QPushButton('🔴 Start Recording')
        self.stop_button = QPushButton('⏹️ Stop Recording')
        self.stop_button.setEnabled(False)
        
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
            }
        """)
        
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
            }
        """)
        
        self.start_button.clicked.connect(self.start_recording)
        self.stop_button.clicked.connect(self.stop_recording)
        
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        
        control_layout.addLayout(button_layout)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Status/Info display
        status_group = QGroupBox("Recording Information")
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel('Status: Ready')
        self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2c3e50;")
        
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(100)
        self.info_text.setStyleSheet("""
            QTextEdit {
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                padding: 5px;
                font-size: 11px;
                background-color: #ecf0f1;
            }
        """)
        self.info_text.setText('Ready to record. Select topics and click "Start Recording".')
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.info_text)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Initialize with topics
        self.refresh_topics()
    
    def refresh_topics(self):
        """Refresh the list of available topics."""
        try:
            self.topic_list.clear()
            topics = self.bag_recorder.get_all_topics()
            
            if topics:
                self.topic_list.addItems(topics)
                self.info_text.append(f'\n✓ Found {len(topics)} topics')
            else:
                self.topic_list.addItem('No topics available')
                self.info_text.append('\n⚠ No topics found. Make sure ROS2 nodes are running.')
        except Exception as e:
            print(f"Error refreshing topics: {e}")
            self.info_text.append(f'\n✗ Error: {e}')
    
    def select_all_topics(self):
        """Select all topics in the list."""
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            if item.text() != 'No topics available':
                item.setSelected(True)
    
    def clear_topic_selection(self):
        """Clear all topic selections."""
        self.topic_list.clearSelection()
    
    def browse_location(self):
        """Open file dialog to select save location."""
        directory = QFileDialog.getExistingDirectory(
            self, 
            'Select Save Location',
            self.location_input.text()
        )
        if directory:
            self.location_input.setText(directory)
    
    def start_recording(self):
        """Start bag recording."""
        selected_items = self.topic_list.selectedItems()
        selected_topics = [item.text() for item in selected_items]
        save_location = self.location_input.text()
        
        # Validate inputs
        if not selected_topics:
            self.status_label.setText('Status: ⚠ Please select at least one topic!')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        if selected_topics == ['No topics available']:
            self.status_label.setText('Status: ⚠ No topics available!')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        if not save_location:
            self.status_label.setText('Status: ⚠ Please specify save location!')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        # Start recording
        success = self.bag_recorder.start_recording(selected_topics, save_location)
        
        if success:
            self.status_label.setText(f'Status: 🔴 Recording {len(selected_topics)} topics...')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #27ae60;")
            
            self.info_text.append(f'\n▶ Recording started')
            self.info_text.append(f'  Topics: {", ".join(selected_topics)}')
            self.info_text.append(f'  Location: {save_location}')
            
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.topic_list.setEnabled(False)
            self.location_input.setEnabled(False)
            
            # Start update timer
            import time
            self.recording_start_time = time.time()
            self.update_timer.start(1000)
        else:
            self.status_label.setText('Status: ✗ Failed to start recording!')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            self.info_text.append('\n✗ Failed to start recording. Check console for errors.')
    
    def stop_recording(self):
        """Stop bag recording."""
        self.update_timer.stop()
        
        bag_path = self.bag_recorder.stop_recording()
        
        if bag_path:
            self.status_label.setText('Status: ✓ Recording stopped')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2980b9;")
            self.info_text.append(f'\n⏹ Recording stopped')
            self.info_text.append(f'  Saved to: {bag_path}')
        else:
            self.status_label.setText('Status: ⚠ Error stopping recording')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
        
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.topic_list.setEnabled(True)
        self.location_input.setEnabled(True)
        self.recording_start_time = None
    
    def update_recording_info(self):
        """Update recording duration display."""
        if self.recording_start_time:
            import time
            duration = int(time.time() - self.recording_start_time)
            minutes = duration // 60
            seconds = duration % 60
            self.status_label.setText(
                f'Status: 🔴 Recording... Duration: {minutes:02d}:{seconds:02d}'
            )
    
    def cleanup(self):
        """Clean up resources."""
        try:
            self.update_timer.stop()
            self.bag_recorder.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
