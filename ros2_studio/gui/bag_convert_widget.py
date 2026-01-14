"""Widget for converting ROS2 bags to CSV format."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
    QLabel, QFileDialog, QLineEdit, QGroupBox,
    QTextEdit, QListWidget, QAbstractItemView, QProgressBar
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from ros2_studio.core.bag_converter import BagConverter
import os


class ConversionThread(QThread):
    """Thread for running bag conversion in background."""
    
    progress = pyqtSignal(str)
    finished = pyqtSignal(dict)
    
    def __init__(self, converter, bag_path, output_dir, topics):
        super().__init__()
        self.converter = converter
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.topics = topics
    
    def run(self):
        """Run the conversion."""
        self.progress.emit("Starting conversion...")
        result = self.converter.convert_bag_to_csv(
            self.bag_path,
            self.output_dir,
            self.topics
        )
        self.finished.emit(result)


class BagConvertWidget(QWidget):
    """Widget for converting ROS2 bags to CSV."""
    
    def __init__(self):
        """Initialize bag convert widget."""
        super().__init__()
        self.bag_converter = BagConverter()
        self.conversion_thread = None
        self.bag_info = None
        self.setup_ui()
    
    def setup_ui(self):
        """Set up the user interface."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # File selection group
        file_group = QGroupBox("Select Bag Folder (containing .db3 and metadata.yaml)")
        file_layout = QVBoxLayout()
        
        file_input_layout = QHBoxLayout()
        file_label = QLabel('Bag Folder:')
        file_label.setStyleSheet("font-weight: bold;")
        
        self.bag_path_input = QLineEdit()
        self.bag_path_input.setPlaceholderText('/path/to/bag/folder')
        
        browse_button = QPushButton('📁 Browse')
        browse_button.clicked.connect(self.browse_bag_file)
        
        load_button = QPushButton('📊 Load Topics')
        load_button.clicked.connect(self.load_bag_topics)
        
        file_input_layout.addWidget(file_label)
        file_input_layout.addWidget(self.bag_path_input)
        file_input_layout.addWidget(browse_button)
        file_input_layout.addWidget(load_button)
        
        file_layout.addLayout(file_input_layout)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Topic selection group
        topic_group = QGroupBox("Select Topics to Convert")
        topic_layout = QVBoxLayout()
        
        topic_header = QHBoxLayout()
        topic_label = QLabel('Available Topics in Bag:')
        topic_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        
        selection_buttons = QHBoxLayout()
        self.select_all_button = QPushButton('Select All')
        self.clear_selection_button = QPushButton('Clear Selection')
        self.select_all_button.clicked.connect(self.select_all_topics)
        self.clear_selection_button.clicked.connect(self.clear_topic_selection)
        self.select_all_button.setEnabled(False)
        self.clear_selection_button.setEnabled(False)
        
        selection_buttons.addWidget(self.select_all_button)
        selection_buttons.addWidget(self.clear_selection_button)
        selection_buttons.addStretch()
        
        topic_header.addWidget(topic_label)
        topic_header.addStretch()
        
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
        
        topic_layout.addLayout(topic_header)
        topic_layout.addWidget(self.topic_list)
        topic_layout.addLayout(selection_buttons)
        
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # Output location group
        output_group = QGroupBox("Output Location")
        output_layout = QHBoxLayout()
        
        output_label = QLabel('CSV Directory:')
        output_label.setStyleSheet("font-weight: bold;")
        
        self.output_input = QLineEdit()
        self.output_input.setPlaceholderText('/path/to/save/csv/files')
        default_path = os.path.expanduser('~/ros2_bags_csv')
        self.output_input.setText(default_path)
        
        output_browse_button = QPushButton('📁 Browse')
        output_browse_button.clicked.connect(self.browse_output_location)
        
        output_layout.addWidget(output_label)
        output_layout.addWidget(self.output_input)
        output_layout.addWidget(output_browse_button)
        
        output_group.setLayout(output_layout)
        layout.addWidget(output_group)
        
        # Convert button
        control_group = QGroupBox("Conversion Controls")
        control_layout = QVBoxLayout()
        
        self.convert_button = QPushButton('🔄 Convert to CSV')
        self.convert_button.setEnabled(False)
        self.convert_button.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
            }
        """)
        self.convert_button.clicked.connect(self.start_conversion)
        
        control_layout.addWidget(self.convert_button)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Status display
        status_group = QGroupBox("Conversion Status")
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel('Status: Ready')
        self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #2c3e50;")
        
        self.status_info = QTextEdit()
        self.status_info.setReadOnly(True)
        self.status_info.setMaximumHeight(120)
        self.status_info.setStyleSheet("""
            QTextEdit {
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                padding: 5px;
                font-size: 11px;
                background-color: #ecf0f1;
            }
        """)
        self.status_info.setText('Select a bag folder (containing .db3 and metadata.yaml) and load topics to begin conversion.')
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.status_info)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
    
    def browse_bag_file(self):
        """Open file dialog to select bag folder."""
        default_path = self.bag_path_input.text() or os.path.expanduser('~/ros2_bags')
        
        directory = QFileDialog.getExistingDirectory(
            self,
            'Select ROS2 Bag Folder (containing .db3 and metadata.yaml)',
            default_path
        )
        
        if directory:
            self.bag_path_input.setText(directory)
    
    def browse_output_location(self):
        """Open file dialog to select output location."""
        directory = QFileDialog.getExistingDirectory(
            self,
            'Select Output Directory',
            self.output_input.text()
        )
        
        if directory:
            self.output_input.setText(directory)
    
    def load_bag_topics(self):
        """Load topics from the selected bag folder."""
        bag_path = self.bag_path_input.text()
        
        if not bag_path:
            self.status_label.setText('Status: ⚠ Please select a bag folder')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        if not os.path.exists(bag_path):
            self.status_label.setText('Status: ⚠ Bag folder not found')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        # Check if it looks like a bag folder
        if not os.path.isdir(bag_path):
            self.status_label.setText('Status: ⚠ Please select a folder, not a file')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        self.status_info.append('\n📂 Loading bag information...')
        
        try:
            self.bag_info = self.bag_converter.get_bag_info(bag_path)
            
            if self.bag_info and self.bag_info['topics']:
                self.topic_list.clear()
                
                for topic in self.bag_info['topics']:
                    topic_type = self.bag_info['topic_types'].get(topic, 'Unknown')
                    display_text = f"{topic} [{topic_type}]"
                    self.topic_list.addItem(display_text)
                
                self.status_label.setText(f'Status: ✓ Found {len(self.bag_info["topics"])} topics')
                self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #27ae60;")
                self.status_info.append(f'✓ Loaded {len(self.bag_info["topics"])} topics from bag')
                
                self.select_all_button.setEnabled(True)
                self.clear_selection_button.setEnabled(True)
                self.convert_button.setEnabled(True)
            else:
                self.status_label.setText('Status: ⚠ No topics found in bag')
                self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
                self.status_info.append('⚠ No topics found in the bag file')
                
        except Exception as e:
            self.status_label.setText('Status: ✗ Error loading bag')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            self.status_info.append(f'✗ Error: {str(e)}')
    
    def select_all_topics(self):
        """Select all topics in the list."""
        for i in range(self.topic_list.count()):
            self.topic_list.item(i).setSelected(True)
    
    def clear_topic_selection(self):
        """Clear all topic selections."""
        self.topic_list.clearSelection()
    
    def start_conversion(self):
        """Start the bag to CSV conversion."""
        selected_items = self.topic_list.selectedItems()
        
        if not selected_items:
            self.status_label.setText('Status: ⚠ Please select topics to convert')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        # Extract topic names (remove type info)
        selected_topics = []
        for item in selected_items:
            topic_full = item.text()
            topic_name = topic_full.split('[')[0].strip()
            selected_topics.append(topic_name)
        
        bag_path = self.bag_path_input.text()
        output_dir = self.output_input.text()
        
        if not output_dir:
            self.status_label.setText('Status: ⚠ Please specify output directory')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            return
        
        # Disable controls
        self.convert_button.setEnabled(False)
        self.topic_list.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 0)  # Indeterminate progress
        
        self.status_label.setText('Status: 🔄 Converting...')
        self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #3498db;")
        self.status_info.append(f'\n🔄 Converting {len(selected_topics)} topics to CSV...')
        
        # Start conversion in thread
        self.conversion_thread = ConversionThread(
            self.bag_converter,
            bag_path,
            output_dir,
            selected_topics
        )
        self.conversion_thread.progress.connect(self.on_progress)
        self.conversion_thread.finished.connect(self.on_conversion_finished)
        self.conversion_thread.start()
    
    def on_progress(self, message):
        """Handle progress updates."""
        self.status_info.append(message)
    
    def on_conversion_finished(self, result):
        """Handle conversion completion."""
        self.progress_bar.setVisible(False)
        self.convert_button.setEnabled(True)
        self.topic_list.setEnabled(True)
        
        if result['success']:
            self.status_label.setText(f'Status: ✓ Conversion Complete')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #27ae60;")
            self.status_info.append(f'\n✓ Successfully converted {result["successful"]}/{result["total"]} topics')
            
            for file_info in result['converted_files']:
                self.status_info.append(f'  • {file_info["topic"]} → {os.path.basename(file_info["file"])}')
            
            if result.get('errors'):
                self.status_info.append(f'\n⚠ Errors:')
                for error in result['errors']:
                    self.status_info.append(f'  • {error}')
            
            self.status_info.append(f'\n📁 Files saved to: {self.output_input.text()}')
        else:
            self.status_label.setText('Status: ✗ Conversion Failed')
            self.status_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
            self.status_info.append(f'\n✗ Error: {result.get("error", "Unknown error")}')
    
    def cleanup(self):
        """Clean up resources."""
        try:
            if self.conversion_thread and self.conversion_thread.isRunning():
                self.conversion_thread.quit()
                self.conversion_thread.wait()
            self.bag_converter.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
