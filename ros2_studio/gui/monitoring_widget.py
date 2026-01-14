"""Performance monitoring widget with graphical display."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QComboBox, 
    QLabel, QPushButton, QGroupBox, QGridLayout
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from ros2_studio.core.performance_monitor import PerformanceMonitor


class MonitoringWidget(QWidget):
    """Widget for monitoring ROS2 topic and node performance."""
    
    def __init__(self):
        """Initialize monitoring widget."""
        super().__init__()
        self.performance_monitor = PerformanceMonitor()
        self.metrics_history = {
            'time': [],
            'cpu': [],
            'memory': [],
            'frequency': []
        }
        self.start_time = None
        
        self.setup_ui()
        
        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_metrics)
        self.timer.start(1000)  # Update every second
    
    def setup_ui(self):
        """Set up the user interface."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Selection area
        selection_group = QGroupBox("Monitor Selection")
        selection_layout = QGridLayout()
        
        # Type selector (Topic/Node)
        type_label = QLabel('Monitor Type:')
        type_label.setStyleSheet("font-weight: bold;")
        self.type_selector = QComboBox()
        self.type_selector.addItems(['Topics', 'Nodes'])
        self.type_selector.currentTextChanged.connect(lambda text: self.update_item_list(text))
        self.type_selector.setMinimumWidth(150)
        
        # Item selector
        item_label = QLabel('Select Item:')
        item_label.setStyleSheet("font-weight: bold;")
        self.item_selector = QComboBox()
        self.item_selector.setMinimumWidth(300)
        self.item_selector.currentTextChanged.connect(self.reset_metrics)
        
        # Refresh button
        self.refresh_button = QPushButton('🔄 Refresh List')
        self.refresh_button.clicked.connect(lambda: self.update_item_list())
        
        selection_layout.addWidget(type_label, 0, 0)
        selection_layout.addWidget(self.type_selector, 0, 1)
        selection_layout.addWidget(item_label, 0, 2)
        selection_layout.addWidget(self.item_selector, 0, 3)
        selection_layout.addWidget(self.refresh_button, 0, 4)
        
        selection_group.setLayout(selection_layout)
        layout.addWidget(selection_group)
        
        # Metrics display area
        metrics_group = QGroupBox("Current Metrics")
        metrics_layout = QGridLayout()
        
        # Create metric labels
        self.cpu_label = self._create_metric_label('CPU Usage:', '0.00%')
        self.memory_label = self._create_metric_label('Memory:', '0.00 MB')
        self.frequency_label = self._create_metric_label('Frequency:', '0.00 Hz')
        self.publishers_label = self._create_metric_label('Publishers:', '0')
        self.subscribers_label = self._create_metric_label('Subscribers:', '0')
        
        metrics_layout.addWidget(self.cpu_label[0], 0, 0)
        metrics_layout.addWidget(self.cpu_label[1], 0, 1)
        metrics_layout.addWidget(self.memory_label[0], 0, 2)
        metrics_layout.addWidget(self.memory_label[1], 0, 3)
        metrics_layout.addWidget(self.frequency_label[0], 1, 0)
        metrics_layout.addWidget(self.frequency_label[1], 1, 1)
        metrics_layout.addWidget(self.publishers_label[0], 1, 2)
        metrics_layout.addWidget(self.publishers_label[1], 1, 3)
        metrics_layout.addWidget(self.subscribers_label[0], 2, 0)
        metrics_layout.addWidget(self.subscribers_label[1], 2, 1)
        
        metrics_group.setLayout(metrics_layout)
        layout.addWidget(metrics_group)
        
        # Matplotlib figure for graphs
        graph_group = QGroupBox("Performance Graphs (Last 60 seconds)")
        graph_layout = QVBoxLayout()
        
        self.figure = Figure(figsize=(12, 6))
        self.canvas = FigureCanvas(self.figure)
        graph_layout.addWidget(self.canvas)
        
        graph_group.setLayout(graph_layout)
        layout.addWidget(graph_group)
        
        # Initialize with topics
        self.update_item_list('Topics')
    
    def _create_metric_label(self, label_text, value_text):
        """Create a pair of labels for metric display."""
        label = QLabel(label_text)
        label.setStyleSheet("font-weight: bold; font-size: 12px;")
        
        value = QLabel(value_text)
        value.setStyleSheet("font-size: 12px; color: #2980b9;")
        
        return (label, value)
    
    def update_item_list(self, monitor_type=None):
        """Update the list of items based on monitor type."""
        if monitor_type is None:
            monitor_type = self.type_selector.currentText()
        
        self.item_selector.clear()
        
        try:
            if monitor_type == 'Topics':
                topics = self.performance_monitor.get_all_topics()
                if topics:
                    self.item_selector.addItems(topics)
                else:
                    self.item_selector.addItem('No topics available')
            else:  # Nodes
                nodes = self.performance_monitor.get_all_nodes()
                if nodes:
                    self.item_selector.addItems(nodes)
                else:
                    self.item_selector.addItem('No nodes available')
        except Exception as e:
            print(f"Error updating item list: {e}")
            import traceback
            traceback.print_exc()
            self.item_selector.addItem('Error loading items')
    
    def reset_metrics(self):
        """Reset metrics history when item changes."""
        self.metrics_history = {
            'time': [],
            'cpu': [],
            'memory': [],
            'frequency': []
        }
        self.start_time = None
        self.figure.clear()
        self.canvas.draw()
    
    def update_metrics(self):
        """Update metrics display and graphs."""
        selected_type = self.type_selector.currentText()
        selected_item = self.item_selector.currentText()
        
        if not selected_item or selected_item in ['No topics available', 'No nodes available', 'Error loading items']:
            return
        
        try:
            # Get metrics
            if selected_type == 'Topics':
                metrics = self.performance_monitor.get_topic_metrics(selected_item)
            else:
                metrics = self.performance_monitor.get_node_metrics(selected_item)
            
            if metrics:
                # Update history
                import time
                current_time = time.time()
                if self.start_time is None:
                    self.start_time = current_time
                
                relative_time = current_time - self.start_time
                self.metrics_history['time'].append(relative_time)
                self.metrics_history['cpu'].append(metrics.get('cpu', 0))
                self.metrics_history['memory'].append(metrics.get('memory', 0))
                self.metrics_history['frequency'].append(metrics.get('frequency', 0))
                
                # Keep only last 60 data points
                if len(self.metrics_history['time']) > 60:
                    for key in self.metrics_history:
                        self.metrics_history[key] = self.metrics_history[key][-60:]
                
                # Update text display
                self.cpu_label[1].setText(f"{metrics.get('cpu', 0):.2f}%")
                self.memory_label[1].setText(f"{metrics.get('memory', 0):.2f} MB")
                self.frequency_label[1].setText(f"{metrics.get('frequency', 0):.2f} Hz")
                
                if selected_type == 'Topics':
                    self.publishers_label[1].setText(f"{metrics.get('publishers', 0)}")
                    self.subscribers_label[1].setText(f"{metrics.get('subscribers', 0)}")
                else:
                    self.publishers_label[1].setText('N/A')
                    self.subscribers_label[1].setText('N/A')
                
                # Update graph
                self.plot_metrics()
        except Exception as e:
            print(f"Error updating metrics: {e}")
    
    def plot_metrics(self):
        """Plot metrics on the graph."""
        try:
            self.figure.clear()
            
            if not self.metrics_history['time']:
                return
            
            # Create subplots
            ax1 = self.figure.add_subplot(311)
            ax2 = self.figure.add_subplot(312)
            ax3 = self.figure.add_subplot(313)
            
            time_data = self.metrics_history['time']
            
            # Plot CPU
            ax1.plot(time_data, self.metrics_history['cpu'], 'b-', linewidth=2)
            ax1.set_ylabel('CPU Usage (%)', fontsize=10, fontweight='bold')
            ax1.set_title('Performance Metrics Over Time', fontsize=12, fontweight='bold')
            ax1.grid(True, alpha=0.3)
            ax1.set_xlim(left=0)
            
            # Plot Memory
            ax2.plot(time_data, self.metrics_history['memory'], 'r-', linewidth=2)
            ax2.set_ylabel('Memory (MB)', fontsize=10, fontweight='bold')
            ax2.grid(True, alpha=0.3)
            ax2.set_xlim(left=0)
            
            # Plot Frequency
            ax3.plot(time_data, self.metrics_history['frequency'], 'g-', linewidth=2)
            ax3.set_xlabel('Time (seconds)', fontsize=10, fontweight='bold')
            ax3.set_ylabel('Frequency (Hz)', fontsize=10, fontweight='bold')
            ax3.grid(True, alpha=0.3)
            ax3.set_xlim(left=0)
            
            self.figure.tight_layout()
            self.canvas.draw()
        except Exception as e:
            print(f"Error plotting metrics: {e}")
    
    def cleanup(self):
        """Clean up resources."""
        try:
            self.timer.stop()
            self.performance_monitor.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
