"""
AutoTM GUI - FTC Otonom Robot Simülasyonu (Grafiksel Arayüz)
Made by Error 24501

2025-2026 DECODE™ Harita Desteği, Servo Kontrol, Ek Motorlar
Tkinter GUI ile kolay kullanım
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle, Circle, FancyArrow
import json
import math


class PIDController:
    """PID kontrolcüsü"""
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
    
    def calculate(self, error: float, dt: float = 0.1) -> float:
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def reset(self):
        self.integral = 0
        self.prev_error = 0


class Servo:
    """Servo motor sınıfı"""
    def __init__(self, name: str, port: int, initial_position: float = 0.0):
        self.name = name
        self.port = port
        self.position = initial_position
        self.min_angle = 0.0
        self.max_angle = 180.0
        self.history = [initial_position]
    
    def set_position(self, position: float):
        self.position = max(0.0, min(1.0, position))
        self.history.append(self.position)
    
    def get_angle(self) -> float:
        return self.min_angle + (self.max_angle - self.min_angle) * self.position


class Motor:
    """Ek motor sınıfı"""
    def __init__(self, name: str, port: int, gear_ratio: float = 1.0):
        self.name = name
        self.port = port
        self.gear_ratio = gear_ratio
        self.power = 0.0
        self.position = 0
        self.target_position = 0
        self.history = []
    
    def set_power(self, power: float):
        self.power = max(-1.0, min(1.0, power))
    
    def move_to_position(self, ticks: int, speed: float = 0.5):
        self.target_position = ticks
        self.power = speed if ticks > self.position else -speed
        self.position = ticks
        self.history.append(self.position)


class Robot:
    """FTC Robot sınıfı"""
    def __init__(self, x: float = 0, y: float = 0, angle: float = 0,
                 wheel_radius: float = 5.0, length: float = 45.0, width: float = 45.0,
                 kp: float = 1.0, ki: float = 0.0, kd: float = 0.1):
        self.x = x
        self.y = y
        self.angle = angle
        self.wheel_radius = wheel_radius
        self.length = length
        self.width = width
        
        self.pid_distance = PIDController(kp, ki, kd)
        self.pid_angle = PIDController(kp, ki, kd)
        
        self.path_x = [x]
        self.path_y = [y]
        self.path_angles = [angle]
        
        self.servos = {}
        self.motors = {}
    
    def add_servo(self, name: str, port: int, initial_position: float = 0.0):
        self.servos[name] = Servo(name, port, initial_position)
    
    def add_motor(self, name: str, port: int, gear_ratio: float = 1.0):
        self.motors[name] = Motor(name, port, gear_ratio)
    
    def control_servo(self, name: str, position: float):
        if name in self.servos:
            self.servos[name].set_position(position)
    
    def control_motor(self, name: str, action: str, value: float):
        if name in self.motors:
            if action == 'power':
                self.motors[name].set_power(value)
            elif action == 'position':
                self.motors[name].move_to_position(int(value))
    
    def move_forward(self, distance: float, steps: int = 50):
        self.pid_distance.reset()
        target_x = self.x + distance * math.sin(math.radians(self.angle))
        target_y = self.y + distance * math.cos(math.radians(self.angle))
        
        for step in range(steps):
            dx = target_x - self.x
            dy = target_y - self.y
            error = math.sqrt(dx**2 + dy**2)
            
            if error < 0.1:
                break
            
            control = self.pid_distance.calculate(error)
            step_size = min(control, distance / steps)
            
            self.x += step_size * math.sin(math.radians(self.angle))
            self.y += step_size * math.cos(math.radians(self.angle))
            
            self.path_x.append(self.x)
            self.path_y.append(self.y)
            self.path_angles.append(self.angle)
        
        self.x = target_x
        self.y = target_y
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        self.path_angles.append(self.angle)
    
    def turn(self, degrees: float, steps: int = 30):
        self.pid_angle.reset()
        target_angle = self.angle + degrees
        
        for step in range(steps):
            error = target_angle - self.angle
            
            if abs(error) < 0.5:
                break
            
            control = self.pid_angle.calculate(error)
            turn_step = min(abs(control), abs(degrees) / steps) * (1 if error > 0 else -1)
            self.angle += turn_step
            
            self.path_x.append(self.x)
            self.path_y.append(self.y)
            self.path_angles.append(self.angle)
        
        self.angle = target_angle
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        self.path_angles.append(self.angle)
    
    def strafe(self, distance: float, direction: str = 'right', steps: int = 50):
        self.pid_distance.reset()
        strafe_angle = self.angle + (90 if direction == 'right' else -90)
        
        target_x = self.x + distance * math.sin(math.radians(strafe_angle))
        target_y = self.y + distance * math.cos(math.radians(strafe_angle))
        
        for step in range(steps):
            dx = target_x - self.x
            dy = target_y - self.y
            error = math.sqrt(dx**2 + dy**2)
            
            if error < 0.1:
                break
            
            control = self.pid_distance.calculate(error)
            step_size = min(control, distance / steps)
            
            self.x += step_size * math.sin(math.radians(strafe_angle))
            self.y += step_size * math.cos(math.radians(strafe_angle))
            
            self.path_x.append(self.x)
            self.path_y.append(self.y)
            self.path_angles.append(self.angle)
        
        self.x = target_x
        self.y = target_y
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        self.path_angles.append(self.angle)
    
    def reset_path(self):
        self.path_x = [self.x]
        self.path_y = [self.y]
        self.path_angles = [self.angle]


class CommandBlock:
    """Komut bloğu"""
    def __init__(self, command_type: str, value: float = 0, name: str = "", extra: dict = None):
        self.type = command_type
        self.value = value
        self.name = name
        self.extra = extra or {}
    
    def execute(self, robot: Robot):
        if self.type == 'forward':
            robot.move_forward(self.value)
        elif self.type == 'backward':
            robot.move_forward(-self.value)
        elif self.type == 'right':
            robot.turn(self.value)
        elif self.type == 'left':
            robot.turn(-self.value)
        elif self.type == 'strafe_right':
            robot.strafe(self.value, 'right')
        elif self.type == 'strafe_left':
            robot.strafe(self.value, 'left')
        elif self.type == 'servo':
            robot.control_servo(self.name, self.value)
        elif self.type == 'motor_power':
            robot.control_motor(self.name, 'power', self.value)
        elif self.type == 'motor_position':
            robot.control_motor(self.name, 'position', self.value)
    
    def __str__(self):
        if self.type == 'forward':
            return f"İleri: {self.value} cm"
        elif self.type == 'backward':
            return f"Geri: {self.value} cm"
        elif self.type == 'right':
            return f"Sağa Dön: {self.value}°"
        elif self.type == 'left':
            return f"Sola Dön: {self.value}°"
        elif self.type == 'strafe_right':
            return f"Sağa Kayma: {self.value} cm"
        elif self.type == 'strafe_left':
            return f"Sola Kayma: {self.value} cm"
        elif self.type == 'servo':
            return f"Servo '{self.name}': {self.value:.2f}"
        elif self.type == 'motor_power':
            return f"Motor '{self.name}' Güç: {self.value:.2f}"
        elif self.type == 'motor_position':
            return f"Motor '{self.name}' Pozisyon: {int(self.value)}"
        return "Bilinmeyen Komut"


class FTCField:
    """FTC DECODE™ Alan"""
    def __init__(self):
        self.width = 365.76
        self.length = 365.76
        
        self.baskets = [
            {'name': 'Kırmızı', 'x': 60, 'y': 60, 'color': 'red'},
            {'name': 'Mavi', 'x': 305, 'y': 305, 'color': 'blue'},
        ]
        
        self.specimen_bars = [
            {'x': 30, 'y': 182.88, 'width': 60, 'height': 10, 'color': 'darkred'},
            {'x': 275, 'y': 182.88, 'width': 60, 'height': 10, 'color': 'darkblue'},
        ]
        
        self.observation_zones = [
            {'x': 0, 'y': 0, 'width': 60, 'height': 60, 'color': 'lightcoral'},
            {'x': 305, 'y': 305, 'width': 60, 'height': 60, 'color': 'lightblue'},
        ]
        
        self.submersibles = [
            {'x': 182.88, 'y': 60, 'radius': 25},
            {'x': 182.88, 'y': 305, 'radius': 25},
        ]
    
    def draw_field(self, ax):
        field_border = Rectangle((0, 0), self.width, self.length, 
                                 fill=False, edgecolor='black', linewidth=3)
        ax.add_patch(field_border)
        
        ax.plot([self.width/2, self.width/2], [0, self.length], 'k--', linewidth=2, alpha=0.3)
        ax.plot([0, self.width], [self.length/2, self.length/2], 'k--', linewidth=2, alpha=0.3)
        
        for basket in self.baskets:
            circle = Circle((basket['x'], basket['y']), 20, 
                          facecolor=basket['color'], edgecolor='black', alpha=0.5)
            ax.add_patch(circle)
        
        for bar in self.specimen_bars:
            rect = Rectangle((bar['x'], bar['y']), bar['width'], bar['height'],
                           facecolor=bar['color'], edgecolor='black', alpha=0.6)
            ax.add_patch(rect)
        
        for zone in self.observation_zones:
            rect = Rectangle((zone['x'], zone['y']), zone['width'], zone['height'],
                           facecolor=zone['color'], edgecolor='black', alpha=0.3, linestyle='--')
            ax.add_patch(rect)
        
        for sub in self.submersibles:
            circle = Circle((sub['x'], sub['y']), sub['radius'], 
                          facecolor='yellow', edgecolor='black', alpha=0.6)
            ax.add_patch(circle)


class AutoTMGUI:
    """AutoTM GUI Ana Sınıfı"""
    def __init__(self, root):
        self.root = root
        self.root.title("AutoTM - FTC DECODE™ 2025-2026 | Made by Error 24501")
        self.root.geometry("1400x800")
        
        self.robot = Robot(60, 60, 0)
        self.commands = []
        self.field = FTCField()
        
        self.setup_ui()
        
    def setup_ui(self):
        """UI Kurulumu"""
        # Ana container
        main_container = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Sol Panel - Kontroller
        left_panel = ttk.Frame(main_container, width=400)
        main_container.add(left_panel, weight=1)
        
        # Sağ Panel - Görselleştirme
        right_panel = ttk.Frame(main_container)
        main_container.add(right_panel, weight=2)
        
        self.setup_left_panel(left_panel)
        self.setup_right_panel(right_panel)
    
    def setup_left_panel(self, parent):
        """Sol panel - Kontroller"""
        # Başlık
        title = ttk.Label(parent, text="🤖 AutoTM Kontrol Paneli", 
                         font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        # Robot Ayarları
        robot_frame = ttk.LabelFrame(parent, text="Robot Ayarları", padding=10)
        robot_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(robot_frame, text="Başlangıç X (cm):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.x_entry = ttk.Entry(robot_frame, width=10)
        self.x_entry.insert(0, "60")
        self.x_entry.grid(row=0, column=1, pady=2)
        
        ttk.Label(robot_frame, text="Başlangıç Y (cm):").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.y_entry = ttk.Entry(robot_frame, width=10)
        self.y_entry.insert(0, "60")
        self.y_entry.grid(row=1, column=1, pady=2)
        
        ttk.Label(robot_frame, text="Açı (°):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.angle_entry = ttk.Entry(robot_frame, width=10)
        self.angle_entry.insert(0, "0")
        self.angle_entry.grid(row=2, column=1, pady=2)
        
        ttk.Button(robot_frame, text="Robot'u Güncelle", 
                  command=self.update_robot).grid(row=3, column=0, columnspan=2, pady=5)
        
        # Hareket Komutları
        move_frame = ttk.LabelFrame(parent, text="Hareket Komutları", padding=10)
        move_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(move_frame, text="Mesafe/Açı:").grid(row=0, column=0, sticky=tk.W)
        self.value_entry = ttk.Entry(move_frame, width=10)
        self.value_entry.insert(0, "50")
        self.value_entry.grid(row=0, column=1)
        
        btn_frame = ttk.Frame(move_frame)
        btn_frame.grid(row=1, column=0, columnspan=2, pady=5)
        
        ttk.Button(btn_frame, text="⬆ İleri", width=12,
                  command=lambda: self.add_command('forward')).pack(side=tk.TOP, pady=2)
        
        row2 = ttk.Frame(btn_frame)
        row2.pack(pady=2)
        ttk.Button(row2, text="⬅ Sola Dön", width=12,
                  command=lambda: self.add_command('left')).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="➡ Sağa Dön", width=12,
                  command=lambda: self.add_command('right')).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(btn_frame, text="⬇ Geri", width=12,
                  command=lambda: self.add_command('backward')).pack(side=tk.TOP, pady=2)
        
        row3 = ttk.Frame(btn_frame)
        row3.pack(pady=2)
        ttk.Button(row3, text="↼ Sola Kay", width=12,
                  command=lambda: self.add_command('strafe_left')).pack(side=tk.LEFT, padx=2)
        ttk.Button(row3, text="⇀ Sağa Kay", width=12,
                  command=lambda: self.add_command('strafe_right')).pack(side=tk.LEFT, padx=2)
        
        # Servo/Motor Kontrol
        component_frame = ttk.LabelFrame(parent, text="Servo/Motor", padding=10)
        component_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(component_frame, text="➕ Servo Ekle", 
                  command=self.add_servo_dialog).pack(fill=tk.X, pady=2)
        ttk.Button(component_frame, text="➕ Motor Ekle", 
                  command=self.add_motor_dialog).pack(fill=tk.X, pady=2)
        
        # Komut Listesi
        cmd_frame = ttk.LabelFrame(parent, text="Komut Listesi", padding=10)
        cmd_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.cmd_listbox = tk.Listbox(cmd_frame, height=10)
        self.cmd_listbox.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        
        scrollbar = ttk.Scrollbar(cmd_frame, command=self.cmd_listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.cmd_listbox.config(yscrollcommand=scrollbar.set)
        
        btn_cmd_frame = ttk.Frame(cmd_frame)
        btn_cmd_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_cmd_frame, text="🗑 Sil", 
                  command=self.delete_command).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_cmd_frame, text="🔄 Temizle", 
                  command=self.clear_commands).pack(side=tk.LEFT, padx=2)
        
        # Aksiyon Butonları
        action_frame = ttk.Frame(parent)
        action_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(action_frame, text="▶ Simülasyon Çalıştır", 
                  command=self.run_simulation, style="Accent.TButton").pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="📊 Görselleştir", 
                  command=self.visualize).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="☕ Java Kodu Üret", 
                  command=self.generate_java).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="💾 Kaydet", 
                  command=self.save_commands).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="📂 Yükle", 
                  command=self.load_commands).pack(fill=tk.X, pady=2)
    
    def setup_right_panel(self, parent):
        """Sağ panel - Görselleştirme"""
        title = ttk.Label(parent, text="📊 Simülasyon Görselleştirme", 
                         font=("Arial", 12, "bold"))
        title.pack(pady=10)
        
        # Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # İlk çizim
        self.draw_initial_field()
    
    def draw_initial_field(self):
        """İlk alan çizimi"""
        self.ax.clear()
        self.field.draw_field(self.ax)
        
        # Robot başlangıç pozisyonu
        self.ax.plot(self.robot.x, self.robot.y, 'go', markersize=15, 
                    label='Robot', markeredgecolor='black', markeredgewidth=2)
        
        self.ax.set_title('AutoTM - DECODE™ 2025-2026\nMade by Error 24501', 
                         fontsize=12, fontweight='bold')
        self.ax.legend()
        self.ax.set_xlim(-20, self.field.width + 20)
        self.ax.set_ylim(-20, self.field.length + 20)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.2)
        
        self.canvas.draw()
    
    def update_robot(self):
        """Robot parametrelerini güncelle"""
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            angle = float(self.angle_entry.get())
            
            self.robot = Robot(x, y, angle)
            messagebox.showinfo("Başarılı", "Robot güncellendi!")
            self.draw_initial_field()
        except ValueError:
            messagebox.showerror("Hata", "Geçersiz değer!")
    
    def add_command(self, cmd_type):
        """Komut ekle"""
        try:
            value = float(self.value_entry.get())
            cmd = CommandBlock(cmd_type, value)
            self.commands.append(cmd)
            self.cmd_listbox.insert(tk.END, str(cmd))
        except ValueError:
            messagebox.showerror("Hata", "Geçersiz değer!")
    
    def delete_command(self):
        """Seçili komutu sil"""
        selection = self.cmd_listbox.curselection()
        if selection:
            index = selection[0]
            self.cmd_listbox.delete(index)
            self.commands.pop(index)
    
    def clear_commands(self):
        """Tüm komutları temizle"""
        if messagebox.askyesno("Onay", "Tüm komutlar silinecek. Emin misiniz?"):
            self.commands.clear()
            self.cmd_listbox.delete(0, tk.END)
    
    def add_servo_dialog(self):
        """Servo ekleme penceresi"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Servo Ekle")
        dialog.geometry("300x200")
        
        ttk.Label(dialog, text="Servo Adı:").pack(pady=5)
        name_entry = ttk.Entry(dialog)
        name_entry.pack(pady=5)
        
        ttk.Label(dialog, text="Port (0-5):").pack(pady=5)
        port_entry = ttk.Entry(dialog)
        port_entry.pack(pady=5)
        
        ttk.Label(dialog, text="Başlangıç Pozisyon (0.0-1.0):").pack(pady=5)
        pos_entry = ttk.Entry(dialog)
        pos_entry.insert(0, "0.0")
        pos_entry.pack(pady=5)
        
        def add():
            try:
                name = name_entry.get()
                port = int(port_entry.get())
                pos = float(pos_entry.get())
                self.robot.add_servo(name, port, pos)
                messagebox.showinfo("Başarılı", f"Servo '{name}' eklendi!")
                dialog.destroy()
            except ValueError:
                messagebox.showerror("Hata", "Geçersiz değer!")
        
        ttk.Button(dialog, text="Ekle", command=add).pack(pady=10)
    
    def add_motor_dialog(self):
        """Motor ekleme penceresi"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Motor Ekle")
        dialog.geometry("300x200")
        
        ttk.Label(dialog, text="Motor Adı:").pack(pady=5)
        name_entry = ttk.Entry(dialog)
        name_entry.pack(pady=5)
        
        ttk.Label(dialog, text="Port (0-7):").pack(pady=5)
        port_entry = ttk.Entry(dialog)
        port_entry.pack(pady=5)
        
        ttk.Label(dialog, text="Gear Ratio:").pack(pady=5)
        ratio_entry = ttk.Entry(dialog)
        ratio_entry.insert(0, "1.0")
        ratio_entry.pack(pady=5)
        
        def add():
            try:
                name = name_entry.get()
                port = int(port_entry.get())
                ratio = float(ratio_entry.get())
                self.robot.add_motor(name, port, ratio)
                messagebox.showinfo("Başarılı", f"Motor '{name}' eklendi!")
                dialog.destroy()
            except ValueError:
                messagebox.showerror("Hata", "Geçersiz değer!")
        
        ttk.Button(dialog, text="Ekle", command=add).pack(pady=10)
    
    def run_simulation(self):
        """Simülasyonu çalıştır"""
        if not self.commands:
            messagebox.showwarning("Uyarı", "Önce komut eklemelisiniz!")
            return
        
        # Robot'u başlangıç konumuna sıfırla
        x = float(self.x_entry.get())
        y = float(self.y_entry.get())
        angle = float(self.angle_entry.get())
        self.robot = Robot(x, y, angle)
        
        # Komutları çalıştır
        for cmd in self.commands:
            cmd.execute(self.robot)
        
        messagebox.showinfo("Başarılı", 
                          f"Simülasyon tamamlandı!\n\n"
                          f"Başlangıç: X={x:.1f}, Y={y:.1f}\n"
                          f"Bitiş: X={self.robot.x:.1f}, Y={self.robot.y:.1f}\n"
                          f"Toplam adım: {len(self.robot.path_x)}")
    
    def visualize(self):
        """Simülasyonu görselleştir"""
        if not self.commands:
            messagebox.showwarning("Uyarı", "Önce komut ekleyip simülasyonu çalıştırmalısınız!")
            return
        
        if len(self.robot.path_x) <= 1:
            if messagebox.askyesno("Simülasyon", "Simülasyon çalıştırılsın mı?"):
                self.run_simulation()
            else:
                return
        
        self.ax.clear()
        self.field.draw_field(self.ax)
        
        # Yolu çiz
        self.ax.plot(self.robot.path_x, self.robot.path_y, 'b-', 
                    linewidth=3, label='Robot Yolu', alpha=0.7)
        
        # Başlangıç ve bitiş
        self.ax.plot(self.robot.path_x[0], self.robot.path_y[0], 'go', 
                    markersize=20, label='Başlangıç', markeredgecolor='black', markeredgewidth=2)
        self.ax.plot(self.robot.path_x[-1], self.robot.path_y[-1], 'ro', 
                    markersize=20, label='Bitiş', markeredgecolor='black', markeredgewidth=2)
        
        # Robot yönleri
        step_interval = max(1, len(self.robot.path_x) // 25)
        for i in range(0, len(self.robot.path_x), step_interval):
            x, y, angle = self.robot.path_x[i], self.robot.path_y[i], self.robot.path_angles[i]
            dx = 15 * math.sin(math.radians(angle))
            dy = 15 * math.cos(math.radians(angle))
            self.ax.arrow(x, y, dx, dy, head_width=5, head_length=5, 
                         fc='darkred', ec='darkred', alpha=0.6, linewidth=2)
        
        # Son robot konumu
        final_angle = self.robot.path_angles[-1]
        rect = Rectangle(
            (self.robot.x - self.robot.width/2, self.robot.y - self.robot.length/2),
            self.robot.width, self.robot.length,
            angle=final_angle, facecolor='orange', alpha=0.7, 
            edgecolor='black', linewidth=3
        )
        self.ax.add_patch(rect)
        
        self.ax.set_title('AutoTM - DECODE™ 2025-2026\nMade by Error 24501', 
                         fontsize=12, fontweight='bold')
        self.ax.legend(loc='upper right')
        self.ax.set_xlim(-20, self.field.width + 20)
        self.ax.set_ylim(-20, self.field.length + 20)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.2)
        
        self.canvas.draw()
    
    def generate_java(self):
        """Java kodu üret"""
        if not self.commands:
            messagebox.showwarning("Uyarı", "Önce komut eklemelisiniz!")
            return
        
        code = self._generate_java_code()
        
        # Yeni pencerede göster
        dialog = tk.Toplevel(self.root)
        dialog.title("Java Kodu - AutoTM")
        dialog.geometry("800x600")
        
        text_widget = scrolledtext.ScrolledText(dialog, wrap=tk.WORD, font=("Courier", 10))
        text_widget.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        text_widget.insert(1.0, code)
        text_widget.config(state=tk.DISABLED)
        
        def save():
            filename = filedialog.asksaveasfilename(
                defaultextension=".java",
                filetypes=[("Java files", "*.java"), ("All files", "*.*")],
                initialfile="AutoTM_DECODE.java"
            )
            if filename:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(code)
                messagebox.showinfo("Başarılı", f"Java kodu kaydedildi:\n{filename}")
        
        ttk.Button(dialog, text="💾 Dosyaya Kaydet", command=save).pack(pady=10)
    
    def _generate_java_code(self) -> str:
        """Java kodu üret (iç metod)"""
        code = """// AutoTM - FTC Otonom Java Kodu (LinearOpMode)
// Made by Error 24501
// DECODE™ 2025-2026 Sezon

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutoTM - DECODE 2025", group="Error24501")
public class AutoTM_DECODE extends LinearOpMode {
    
    // Sürüş motorları
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    
"""
        
        # Servo tanımlamaları
        if self.robot.servos:
            code += "    // Servo'lar\n"
            for name in self.robot.servos:
                code += f"    private Servo {name}Servo;\n"
            code += "\n"
        
        # Motor tanımlamaları
        if self.robot.motors:
            code += "    // Ek motorlar\n"
            for name in self.robot.motors:
                code += f"    private DcMotor {name}Motor;\n"
            code += "\n"
        
        code += f"""    // Sabitler
    private static final double WHEEL_RADIUS_CM = {self.robot.wheel_radius};
    private static final int TICKS_PER_REV = 1120;
    
    @Override
    public void runOpMode() {{
        // Motorları başlat
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        
"""
        
        # Servo başlatma
        if self.robot.servos:
            code += "        // Servo'ları başlat\n"
            for name in self.robot.servos:
                code += f"        {name}Servo = hardwareMap.get(Servo.class, \"{name}\");\n"
            code += "\n"
        
        # Motor başlatma
        if self.robot.motors:
            code += "        // Ek motorları başlat\n"
            for name in self.robot.motors:
                code += f"        {name}Motor = hardwareMap.get(DcMotor.class, \"{name}\");\n"
                code += f"        {name}Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);\n"
                code += f"        {name}Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);\n"
            code += "\n"
        
        code += """        resetEncoders();
        
        telemetry.addData("Status", "Hazır - AutoTM by Error 24501");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // ===== OTONOM KOMUTLAR =====
"""
        
        # Komutları Java'ya çevir
        for i, cmd in enumerate(self.commands):
            code += f"            // {i+1}. {cmd}\n"
            
            if cmd.type == 'forward':
                code += f"            moveForward({cmd.value});\n"
            elif cmd.type == 'backward':
                code += f"            moveBackward({cmd.value});\n"
            elif cmd.type == 'right':
                code += f"            turnRight({cmd.value});\n"
            elif cmd.type == 'left':
                code += f"            turnLeft({cmd.value});\n"
            elif cmd.type == 'strafe_right':
                code += f"            strafeRight({cmd.value});\n"
            elif cmd.type == 'strafe_left':
                code += f"            strafeLeft({cmd.value});\n"
            elif cmd.type == 'servo':
                code += f"            {cmd.name}Servo.setPosition({cmd.value});\n"
            elif cmd.type == 'motor_power':
                code += f"            {cmd.name}Motor.setPower({cmd.value});\n"
            elif cmd.type == 'motor_position':
                code += f"            moveMotorToPosition({cmd.name}Motor, {int(cmd.value)});\n"
            
            code += "\n"
        
        code += """            
            telemetry.addData("Status", "Tamamlandı!");
            telemetry.update();
        }
    }
    
    // ===== HAREKET FONKSİYONLARI =====
    
    private void moveForward(double distanceCm) {
        int targetTicks = cmToTicks(distanceCm);
        resetEncoders();
        setTargetPosition(targetTicks, targetTicks, targetTicks, targetTicks);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(0.5, 0.5, 0.5, 0.5);
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("İleri", "%.1f cm", distanceCm);
            telemetry.update();
        }
        stopMotors();
    }
    
    private void moveBackward(double distanceCm) {
        moveForward(-distanceCm);
    }
    
    private void turnRight(double degrees) {
        turnByDegrees(degrees);
    }
    
    private void turnLeft(double degrees) {
        turnByDegrees(-degrees);
    }
    
    private void turnByDegrees(double degrees) {
        double wheelBase = 35.0;
        double arcLength = Math.PI * wheelBase * degrees / 180.0;
        int turnTicks = cmToTicks(arcLength);
        
        resetEncoders();
        
        if (degrees > 0) {
            setTargetPosition(turnTicks, -turnTicks, turnTicks, -turnTicks);
        } else {
            setTargetPosition(-turnTicks, turnTicks, -turnTicks, turnTicks);
        }
        
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(0.3, 0.3, 0.3, 0.3);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Dönüş", "%.1f°", degrees);
            telemetry.update();
        }
        
        stopMotors();
    }
    
    private void strafeRight(double distanceCm) {
        int targetTicks = cmToTicks(distanceCm);
        resetEncoders();
        setTargetPosition(-targetTicks, targetTicks, targetTicks, -targetTicks);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(0.5, 0.5, 0.5, 0.5);
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Sağa Kayma", "%.1f cm", distanceCm);
            telemetry.update();
        }
        stopMotors();
    }
    
    private void strafeLeft(double distanceCm) {
        int targetTicks = cmToTicks(distanceCm);
        resetEncoders();
        setTargetPosition(targetTicks, -targetTicks, -targetTicks, targetTicks);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(0.5, 0.5, 0.5, 0.5);
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Sola Kayma", "%.1f cm", distanceCm);
            telemetry.update();
        }
        stopMotors();
    }
    
    private void moveMotorToPosition(DcMotor motor, int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);
        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Motor", "%d / %d", motor.getCurrentPosition(), targetPosition);
            telemetry.update();
        }
        motor.setPower(0);
    }
    
    // ===== YARDIMCI FONKSİYONLAR =====
    
    private int cmToTicks(double cm) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS_CM;
        return (int) ((cm / circumference) * TICKS_PER_REV);
    }
    
    private void resetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    private void setTargetPosition(int lf, int rf, int lb, int rb) {
        leftFrontMotor.setTargetPosition(lf);
        rightFrontMotor.setTargetPosition(rf);
        leftBackMotor.setTargetPosition(lb);
        rightBackMotor.setTargetPosition(rb);
    }
    
    private void setRunMode(DcMotor.RunMode mode) {
        leftFrontMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
        leftBackMotor.setMode(mode);
        rightBackMotor.setMode(mode);
    }
    
    private void setPower(double lf, double rf, double lb, double rb) {
        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftBackMotor.setPower(lb);
        rightBackMotor.setPower(rb);
    }
    
    private void stopMotors() {
        setPower(0, 0, 0, 0);
    }
    
    private boolean motorsAreBusy() {
        return leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || 
               leftBackMotor.isBusy() || rightBackMotor.isBusy();
    }
}
"""
        return code
    
    def save_commands(self):
        """Komutları kaydet"""
        if not self.commands:
            messagebox.showwarning("Uyarı", "Kaydedilecek komut yok!")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialfile="commands.json"
        )
        
        if filename:
            data = {
                'commands': [
                    {
                        'type': cmd.type, 
                        'value': cmd.value, 
                        'name': cmd.name,
                        'extra': cmd.extra
                    } for cmd in self.commands
                ],
                'robot_config': {
                    'x': float(self.x_entry.get()),
                    'y': float(self.y_entry.get()),
                    'angle': float(self.angle_entry.get())
                }
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            messagebox.showinfo("Başarılı", f"Komutlar kaydedildi:\n{filename}")
    
    def load_commands(self):
        """Komutları yükle"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                # Komutları yükle
                self.commands = [
                    CommandBlock(cmd['type'], cmd['value'], cmd.get('name', ''), cmd.get('extra', {}))
                    for cmd in data['commands']
                ]
                
                # Listbox'ı güncelle
                self.cmd_listbox.delete(0, tk.END)
                for cmd in self.commands:
                    self.cmd_listbox.insert(tk.END, str(cmd))
                
                # Robot konfigürasyonunu yükle
                if 'robot_config' in data:
                    config = data['robot_config']
                    self.x_entry.delete(0, tk.END)
                    self.x_entry.insert(0, str(config['x']))
                    self.y_entry.delete(0, tk.END)
                    self.y_entry.insert(0, str(config['y']))
                    self.angle_entry.delete(0, tk.END)
                    self.angle_entry.insert(0, str(config['angle']))
                
                messagebox.showinfo("Başarılı", 
                                  f"{len(self.commands)} komut yüklendi:\n{filename}")
                
            except Exception as e:
                messagebox.showerror("Hata", f"Dosya yüklenemedi:\n{str(e)}")


def main():
    """Ana program"""
    root = tk.Tk()
    
    # Tema ayarları
    style = ttk.Style()
    style.theme_use('clam')
    
    app = AutoTMGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
