"""GUI widget for ROS2 system dashboard."""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QGroupBox, QTableWidget, QTableWidgetItem,
    QTabWidget, QProgressBar, QGridLayout
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from ros2_studio.core.system_dashboard import SystemDashboard


class SystemDashboardWidget(QWidget):
    """Widget for ROS2 system dashboard and diagnostics."""
    
    def __init__(self):
        """Initialize dashboard widget."""
        super().__init__()
        self.dashboard = SystemDashboard()
        
        # History for graphs
        self.cpu_history = []
        self.memory_history = []
        self.network_history = {'sent': [], 'recv': []}
        
        self.setup_ui()
        
        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_dashboard)
        self.timer.start(1000)  # Update every second
    
    def setup_ui(self):
        """Set up the user interface."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Header with refresh button
        header_layout = QHBoxLayout()
        title = QLabel('📊 System Dashboard')
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        
        self.refresh_button = QPushButton('🔄 Refresh')
        self.refresh_button.clicked.connect(self.update_dashboard)
        
        header_layout.addWidget(title)
        header_layout.addStretch()
        header_layout.addWidget(self.refresh_button)
        layout.addLayout(header_layout)
        
        # System Overview Cards
        overview_layout = QHBoxLayout()
        
        # CPU Card
        self.cpu_card = self._create_metric_card('💻 CPU', '0%', QColor(52, 152, 219))
        overview_layout.addWidget(self.cpu_card)
        
        # Memory Card
        self.memory_card = self._create_metric_card('🧠 Memory', '0 GB', QColor(46, 204, 113))
        overview_layout.addWidget(self.memory_card)
        
        # Nodes Card
        self.nodes_card = self._create_metric_card('🔷 Nodes', '0', QColor(155, 89, 182))
        overview_layout.addWidget(self.nodes_card)
        
        # Topics Card
        self.topics_card = self._create_metric_card('📡 Topics', '0', QColor(230, 126, 34))
        overview_layout.addWidget(self.topics_card)
        
        layout.addLayout(overview_layout)
        
        # Tabs for detailed views
        self.tabs = QTabWidget()
        
        # System Resources Tab
        self.resources_tab = self._create_resources_tab()
        self.tabs.addTab(self.resources_tab, '📈 System Resources')
        
        # ROS2 Entities Tab
        self.entities_tab = self._create_entities_tab()
        self.tabs.addTab(self.entities_tab, '🔷 ROS2 Entities')
        
        # Network Tab
        self.network_tab = self._create_network_tab()
        self.tabs.addTab(self.network_tab, '🌐 Network')
        
        # Processes Tab
        self.processes_tab = self._create_processes_tab()
        self.tabs.addTab(self.processes_tab, '⚙️ Processes')
        
        layout.addWidget(self.tabs)
    
    def _create_metric_card(self, title, value, color):
        """Create a metric display card."""
        card = QGroupBox()
        card.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid {color.name()};
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
                background-color: {color.name()}20;
            }}
        """)
        
        layout = QVBoxLayout()
        
        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 14px; font-weight: bold; border: none;")
        
        value_label = QLabel(value)
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setStyleSheet("font-size: 24px; font-weight: bold; border: none;")
        value_label.setObjectName('value')
        
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        card.setLayout(layout)
        
        return card
    
    def _create_resources_tab(self):
        """Create system resources tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Progress bars
        bars_group = QGroupBox("Resource Usage")
        bars_layout = QGridLayout()
        
        # CPU progress
        bars_layout.addWidget(QLabel('CPU:'), 0, 0)
        self.cpu_progress = QProgressBar()
        self.cpu_progress.setStyleSheet("QProgressBar::chunk { background-color: #3498db; }")
        bars_layout.addWidget(self.cpu_progress, 0, 1)
        self.cpu_value_label = QLabel('0%')
        bars_layout.addWidget(self.cpu_value_label, 0, 2)
        
        # Memory progress
        bars_layout.addWidget(QLabel('Memory:'), 1, 0)
        self.memory_progress = QProgressBar()
        self.memory_progress.setStyleSheet("QProgressBar::chunk { background-color: #2ecc71; }")
        bars_layout.addWidget(self.memory_progress, 1, 1)
        self.memory_value_label = QLabel('0 GB / 0 GB')
        bars_layout.addWidget(self.memory_value_label, 1, 2)
        
        # Disk progress
        bars_layout.addWidget(QLabel('Disk:'), 2, 0)
        self.disk_progress = QProgressBar()
        self.disk_progress.setStyleSheet("QProgressBar::chunk { background-color: #e67e22; }")
        bars_layout.addWidget(self.disk_progress, 2, 1)
        self.disk_value_label = QLabel('0 GB / 0 GB')
        bars_layout.addWidget(self.disk_value_label, 2, 2)
        
        bars_group.setLayout(bars_layout)
        layout.addWidget(bars_group)
        
        # Graphs
        graph_group = QGroupBox("Resource History")
        graph_layout = QVBoxLayout()
        
        self.resources_figure = Figure(figsize=(10, 4))
        self.resources_canvas = FigureCanvas(self.resources_figure)
        graph_layout.addWidget(self.resources_canvas)
        
        graph_group.setLayout(graph_layout)
        layout.addWidget(graph_group)
        
        return tab
    
    def _create_entities_tab(self):
        """Create ROS2 entities tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Topics table
        topics_group = QGroupBox("Topics Overview")
        topics_layout = QVBoxLayout()
        
        self.topics_table = QTableWidget()
        self.topics_table.setColumnCount(4)
        self.topics_table.setHorizontalHeaderLabels(['Topic Name', 'Type', 'Publishers', 'Subscribers'])
        self.topics_table.horizontalHeader().setStretchLastSection(True)
        topics_layout.addWidget(self.topics_table)
        
        topics_group.setLayout(topics_layout)
        layout.addWidget(topics_group)
        
        # Nodes table
        nodes_group = QGroupBox("Nodes Overview")
        nodes_layout = QVBoxLayout()
        
        self.nodes_table = QTableWidget()
        self.nodes_table.setColumnCount(2)
        self.nodes_table.setHorizontalHeaderLabels(['Node Name', 'Namespace'])
        self.nodes_table.horizontalHeader().setStretchLastSection(True)
        nodes_layout.addWidget(self.nodes_table)
        
        nodes_group.setLayout(nodes_layout)
        layout.addWidget(nodes_group)
        
        return tab
    
    def _create_network_tab(self):
        """Create network statistics tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Network stats
        stats_group = QGroupBox("Network Statistics")
        stats_layout = QGridLayout()
        
        self.net_sent_label = QLabel('Sent: 0 MB')
        self.net_recv_label = QLabel('Received: 0 MB')
        self.net_packets_sent_label = QLabel('Packets Sent: 0')
        self.net_packets_recv_label = QLabel('Packets Received: 0')
        
        stats_layout.addWidget(self.net_sent_label, 0, 0)
        stats_layout.addWidget(self.net_recv_label, 0, 1)
        stats_layout.addWidget(self.net_packets_sent_label, 1, 0)
        stats_layout.addWidget(self.net_packets_recv_label, 1, 1)
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        # Interfaces
        interfaces_group = QGroupBox("Network Interfaces")
        interfaces_layout = QVBoxLayout()
        
        self.interfaces_table = QTableWidget()
        self.interfaces_table.setColumnCount(3)
        self.interfaces_table.setHorizontalHeaderLabels(['Interface', 'Status', 'IP Address'])
        self.interfaces_table.horizontalHeader().setStretchLastSection(True)
        interfaces_layout.addWidget(self.interfaces_table)
        
        interfaces_group.setLayout(interfaces_layout)
        layout.addWidget(interfaces_group)
        
        # Network graph
        graph_group = QGroupBox("Network Traffic")
        graph_layout = QVBoxLayout()
        
        self.network_figure = Figure(figsize=(10, 3))
        self.network_canvas = FigureCanvas(self.network_figure)
        graph_layout.addWidget(self.network_canvas)
        
        graph_group.setLayout(graph_layout)
        layout.addWidget(graph_group)
        
        return tab
    
    def _create_processes_tab(self):
        """Create ROS processes tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        info_label = QLabel('Top ROS-related processes by CPU usage:')
        info_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(info_label)
        
        self.processes_table = QTableWidget()
        self.processes_table.setColumnCount(4)
        self.processes_table.setHorizontalHeaderLabels(['PID', 'Name', 'CPU %', 'Memory %'])
        self.processes_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.processes_table)
        
        return tab
    
    def update_dashboard(self):
        """Update all dashboard information."""
        try:
            overview = self.dashboard.get_system_overview()
            if overview:
                self._update_overview_cards(overview)
                self._update_resources_tab(overview)
                self._update_network_tab(overview)
            
            # Update tables
            self._update_entities_tab()
            self._update_processes_tab()
            
        except Exception as e:
            print(f"Error updating dashboard: {e}")
    
    def _update_overview_cards(self, overview):
        """Update overview metric cards."""
        # CPU
        cpu_value = self.cpu_card.findChild(QLabel, 'value')
        if cpu_value:
            cpu_value.setText(f"{overview['cpu']['average']:.1f}%")
        
        # Memory
        memory_value = self.memory_card.findChild(QLabel, 'value')
        if memory_value:
            memory_value.setText(f"{overview['memory']['used']:.1f} GB")
        
        # Nodes
        nodes_value = self.nodes_card.findChild(QLabel, 'value')
        if nodes_value:
            nodes_value.setText(str(overview['ros2']['nodes_count']))
        
        # Topics
        topics_value = self.topics_card.findChild(QLabel, 'value')
        if topics_value:
            topics_value.setText(str(overview['ros2']['topics_count']))
    
    def _update_resources_tab(self, overview):
        """Update resources tab."""
        # Progress bars
        cpu_percent = int(overview['cpu']['average'])
        self.cpu_progress.setValue(cpu_percent)
        self.cpu_value_label.setText(f"{overview['cpu']['average']:.1f}%")
        
        memory_percent = int(overview['memory']['percent'])
        self.memory_progress.setValue(memory_percent)
        self.memory_value_label.setText(
            f"{overview['memory']['used']:.1f} GB / {overview['memory']['total']:.1f} GB"
        )
        
        disk_percent = int(overview['disk']['percent'])
        self.disk_progress.setValue(disk_percent)
        self.disk_value_label.setText(
            f"{overview['disk']['used']:.1f} GB / {overview['disk']['total']:.1f} GB"
        )
        
        # Update history
        self.cpu_history.append(overview['cpu']['average'])
        self.memory_history.append(overview['memory']['percent'])
        
        if len(self.cpu_history) > 60:
            self.cpu_history = self.cpu_history[-60:]
            self.memory_history = self.memory_history[-60:]
        
        # Plot graphs
        self._plot_resources()
    
    def _plot_resources(self):
        """Plot resource usage graphs."""
        self.resources_figure.clear()
        
        if len(self.cpu_history) > 1:
            ax1 = self.resources_figure.add_subplot(211)
            ax1.plot(self.cpu_history, 'b-', linewidth=2)
            ax1.set_ylabel('CPU %', fontweight='bold')
            ax1.set_title('System Resources', fontweight='bold')
            ax1.grid(True, alpha=0.3)
            ax1.set_ylim(0, 100)
            
            ax2 = self.resources_figure.add_subplot(212)
            ax2.plot(self.memory_history, 'g-', linewidth=2)
            ax2.set_xlabel('Time (seconds)', fontweight='bold')
            ax2.set_ylabel('Memory %', fontweight='bold')
            ax2.grid(True, alpha=0.3)
            ax2.set_ylim(0, 100)
            
            self.resources_figure.tight_layout()
        
        self.resources_canvas.draw()
    
    def _update_entities_tab(self):
        """Update ROS2 entities tables."""
        # Topics
        topic_stats = self.dashboard.get_topic_statistics()
        self.topics_table.setRowCount(len(topic_stats))
        
        for i, topic in enumerate(topic_stats):
            self.topics_table.setItem(i, 0, QTableWidgetItem(topic['name']))
            self.topics_table.setItem(i, 1, QTableWidgetItem(topic['type'].split('/')[-1]))
            self.topics_table.setItem(i, 2, QTableWidgetItem(str(topic['publishers'])))
            self.topics_table.setItem(i, 3, QTableWidgetItem(str(topic['subscribers'])))
        
        # Nodes
        node_details = self.dashboard.get_node_details()
        self.nodes_table.setRowCount(len(node_details))
        
        for i, node in enumerate(node_details):
            self.nodes_table.setItem(i, 0, QTableWidgetItem(node['name']))
            self.nodes_table.setItem(i, 1, QTableWidgetItem(node['namespace']))
    
    def _update_network_tab(self, overview):
        """Update network tab."""
        network = overview['network']
        
        self.net_sent_label.setText(f"Sent: {network['bytes_sent']:.2f} MB")
        self.net_recv_label.setText(f"Received: {network['bytes_recv']:.2f} MB")
        self.net_packets_sent_label.setText(f"Packets Sent: {network['packets_sent']:,}")
        self.net_packets_recv_label.setText(f"Packets Received: {network['packets_recv']:,}")
        
        # Interfaces
        interfaces = self.dashboard.get_network_interfaces()
        self.interfaces_table.setRowCount(len(interfaces))
        
        for i, (name, info) in enumerate(interfaces.items()):
            self.interfaces_table.setItem(i, 0, QTableWidgetItem(name))
            self.interfaces_table.setItem(i, 1, 
                QTableWidgetItem('✓ UP' if info['is_up'] else '✗ DOWN'))
            self.interfaces_table.setItem(i, 2, QTableWidgetItem(info['ip_address']))
    
    def _update_processes_tab(self):
        """Update processes tab."""
        processes = self.dashboard.get_process_info()
        self.processes_table.setRowCount(len(processes))
        
        for i, proc in enumerate(processes):
            self.processes_table.setItem(i, 0, QTableWidgetItem(str(proc['pid'])))
            self.processes_table.setItem(i, 1, QTableWidgetItem(proc['name']))
            self.processes_table.setItem(i, 2, QTableWidgetItem(f"{proc['cpu']:.1f}"))
            self.processes_table.setItem(i, 3, QTableWidgetItem(f"{proc['memory']:.1f}"))
    
    def cleanup(self):
        """Clean up resources."""
        try:
            self.timer.stop()
            self.dashboard.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
