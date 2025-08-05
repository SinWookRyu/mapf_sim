#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool
from geometry_msgs.msg import Point
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import numpy as np

class GUIControlNode(Node):
    def __init__(self):
        super().__init__('gui_control_node')
        
        # 파라미터 퍼블리셔
        self.num_agents_pub = self.create_publisher(Int32, '/num_agents', 10)
        self.world_width_pub = self.create_publisher(Float32, '/world_width', 10)
        self.world_height_pub = self.create_publisher(Float32, '/world_height', 10)
        self.collision_distance_pub = self.create_publisher(Float32, '/collision_distance', 10)
        self.update_rate_pub = self.create_publisher(Float32, '/update_rate', 10)
        self.reset_sim_pub = self.create_publisher(Bool, '/reset_simulation', 10)
        
        # 충돌 영역 추가 퍼블리셔
        self.add_collision_zone_pub = self.create_publisher(Point, '/add_collision_zone', 10)
        self.clear_collision_zones_pub = self.create_publisher(Bool, '/clear_collision_zones', 10)
        
        # GUI 스레드 시작
        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()
        
        self.get_logger().info('GUI 컨트롤 노드가 시작되었습니다.')
    
    def create_gui(self):
        """GUI 생성"""
        self.root = tk.Tk()
        self.root.title("MAPF 시뮬레이터 제어판")
        self.root.geometry("400x600")
        
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 에이전트 설정 섹션
        agent_frame = ttk.LabelFrame(main_frame, text="에이전트 설정", padding="5")
        agent_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(agent_frame, text="에이전트 수:").grid(row=0, column=0, sticky=tk.W)
        self.num_agents_var = tk.IntVar(value=3)
        num_agents_spin = ttk.Spinbox(agent_frame, from_=1, to=10, textvariable=self.num_agents_var, width=10)
        num_agents_spin.grid(row=0, column=1, padx=5)
        ttk.Button(agent_frame, text="적용", command=self.update_num_agents).grid(row=0, column=2, padx=5)
        
        # 월드 설정 섹션
        world_frame = ttk.LabelFrame(main_frame, text="월드 설정", padding="5")
        world_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(world_frame, text="월드 너비:").grid(row=0, column=0, sticky=tk.W)
        self.world_width_var = tk.DoubleVar(value=20.0)
        world_width_spin = ttk.Spinbox(world_frame, from_=10.0, to=50.0, increment=1.0, 
                                      textvariable=self.world_width_var, width=10)
        world_width_spin.grid(row=0, column=1, padx=5)
        ttk.Button(world_frame, text="적용", command=self.update_world_size).grid(row=0, column=2, padx=5)
        
        ttk.Label(world_frame, text="월드 높이:").grid(row=1, column=0, sticky=tk.W)
        self.world_height_var = tk.DoubleVar(value=20.0)
        world_height_spin = ttk.Spinbox(world_frame, from_=10.0, to=50.0, increment=1.0, 
                                       textvariable=self.world_height_var, width=10)
        world_height_spin.grid(row=1, column=1, padx=5)
        
        # 충돌 설정 섹션
        collision_frame = ttk.LabelFrame(main_frame, text="충돌 설정", padding="5")
        collision_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(collision_frame, text="충돌 거리:").grid(row=0, column=0, sticky=tk.W)
        self.collision_distance_var = tk.DoubleVar(value=1.5)
        collision_distance_spin = ttk.Spinbox(collision_frame, from_=0.5, to=5.0, increment=0.1, 
                                            textvariable=self.collision_distance_var, width=10)
        collision_distance_spin.grid(row=0, column=1, padx=5)
        ttk.Button(collision_frame, text="적용", command=self.update_collision_distance).grid(row=0, column=2, padx=5)
        
        # 시뮬레이션 설정 섹션
        sim_frame = ttk.LabelFrame(main_frame, text="시뮬레이션 설정", padding="5")
        sim_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(sim_frame, text="업데이트 속도 (Hz):").grid(row=0, column=0, sticky=tk.W)
        self.update_rate_var = tk.DoubleVar(value=10.0)
        update_rate_spin = ttk.Spinbox(sim_frame, from_=1.0, to=30.0, increment=1.0, 
                                      textvariable=self.update_rate_var, width=10)
        update_rate_spin.grid(row=0, column=1, padx=5)
        ttk.Button(sim_frame, text="적용", command=self.update_rate).grid(row=0, column=2, padx=5)
        
        # 충돌 영역 관리 섹션
        zone_frame = ttk.LabelFrame(main_frame, text="충돌 영역 관리", padding="5")
        zone_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(zone_frame, text="X 위치:").grid(row=0, column=0, sticky=tk.W)
        self.zone_x_var = tk.DoubleVar(value=0.0)
        zone_x_spin = ttk.Spinbox(zone_frame, from_=-10.0, to=10.0, increment=0.5, 
                                  textvariable=self.zone_x_var, width=10)
        zone_x_spin.grid(row=0, column=1, padx=5)
        
        ttk.Label(zone_frame, text="Y 위치:").grid(row=1, column=0, sticky=tk.W)
        self.zone_y_var = tk.DoubleVar(value=0.0)
        zone_y_spin = ttk.Spinbox(zone_frame, from_=-10.0, to=10.0, increment=0.5, 
                                  textvariable=self.zone_y_var, width=10)
        zone_y_spin.grid(row=1, column=1, padx=5)
        
        ttk.Label(zone_frame, text="반지름:").grid(row=2, column=0, sticky=tk.W)
        self.zone_radius_var = tk.DoubleVar(value=2.0)
        zone_radius_spin = ttk.Spinbox(zone_frame, from_=0.5, to=10.0, increment=0.5, 
                                       textvariable=self.zone_radius_var, width=10)
        zone_radius_spin.grid(row=2, column=1, padx=5)
        
        ttk.Button(zone_frame, text="충돌 영역 추가", command=self.add_collision_zone).grid(row=3, column=0, columnspan=2, pady=5)
        ttk.Button(zone_frame, text="충돌 영역 모두 제거", command=self.clear_collision_zones).grid(row=4, column=0, columnspan=2, pady=5)
        
        # 제어 버튼 섹션
        control_frame = ttk.LabelFrame(main_frame, text="시뮬레이션 제어", padding="5")
        control_frame.grid(row=5, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(control_frame, text="시뮬레이션 리셋", command=self.reset_simulation).grid(row=0, column=0, columnspan=2, pady=5)
        
        # 상태 표시 섹션
        status_frame = ttk.LabelFrame(main_frame, text="상태 정보", padding="5")
        status_frame.grid(row=6, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.status_text = tk.Text(status_frame, height=8, width=45)
        self.status_text.grid(row=0, column=0, pady=5)
        
        # 스크롤바 추가
        scrollbar = ttk.Scrollbar(status_frame, orient="vertical", command=self.status_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.status_text.configure(yscrollcommand=scrollbar.set)
        
        # 초기 상태 메시지
        self.log_status("GUI가 시작되었습니다.")
        self.log_status("시뮬레이터를 시작하려면 'ros2 run mapf_simulator mapf_simulator_node'를 실행하세요.")
        
        # GUI 메인 루프
        self.root.mainloop()
    
    def log_status(self, message: str):
        """상태 로그 추가"""
        self.status_text.insert(tk.END, f"{message}\n")
        self.status_text.see(tk.END)
    
    def update_num_agents(self):
        """에이전트 수 업데이트"""
        num_agents = self.num_agents_var.get()
        msg = Int32()
        msg.data = num_agents
        self.num_agents_pub.publish(msg)
        self.log_status(f"에이전트 수를 {num_agents}로 설정했습니다.")
    
    def update_world_size(self):
        """월드 크기 업데이트"""
        width = self.world_width_var.get()
        height = self.world_height_var.get()
        
        width_msg = Float32()
        width_msg.data = width
        self.world_width_pub.publish(width_msg)
        
        height_msg = Float32()
        height_msg.data = height
        self.world_height_pub.publish(height_msg)
        
        self.log_status(f"월드 크기를 {width}x{height}로 설정했습니다.")
    
    def update_collision_distance(self):
        """충돌 거리 업데이트"""
        distance = self.collision_distance_var.get()
        msg = Float32()
        msg.data = distance
        self.collision_distance_pub.publish(msg)
        self.log_status(f"충돌 거리를 {distance}로 설정했습니다.")
    
    def update_rate(self):
        """업데이트 속도 변경"""
        rate = self.update_rate_var.get()
        msg = Float32()
        msg.data = rate
        self.update_rate_pub.publish(msg)
        self.log_status(f"업데이트 속도를 {rate}Hz로 설정했습니다.")
    
    def add_collision_zone(self):
        """충돌 영역 추가"""
        x = self.zone_x_var.get()
        y = self.zone_y_var.get()
        radius = self.zone_radius_var.get()
        
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = radius  # z 필드를 반지름으로 사용
        
        self.add_collision_zone_pub.publish(msg)
        self.log_status(f"충돌 영역을 추가했습니다: 위치({x}, {y}), 반지름 {radius}")
    
    def clear_collision_zones(self):
        """충돌 영역 모두 제거"""
        msg = Bool()
        msg.data = True
        self.clear_collision_zones_pub.publish(msg)
        self.log_status("모든 충돌 영역을 제거했습니다.")
    
    def reset_simulation(self):
        """시뮬레이션 리셋"""
        msg = Bool()
        msg.data = True
        self.reset_sim_pub.publish(msg)
        self.log_status("시뮬레이션을 리셋했습니다.")

def main(args=None):
    rclpy.init(args=args)
    
    gui_node = GUIControlNode()
    
    try:
        rclpy.spin(gui_node)
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 