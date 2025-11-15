"""
AutoTM - FTC Otonom Robot Simülasyonu
Made by Error 24501

2025-2026 DECODE™ Harita Desteği, Servo Kontrol, Ek Motorlar
Python ile blok tabanlı hareket planlaması, PID kontrolü ve Java kod üretimi
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyArrow
from matplotlib.image import imread
import json
from typing import List, Tuple, Dict
import math
import os


class PIDController:
    """PID kontrolcüsü - mesafe ve açı kontrolü için"""
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
    
    def calculate(self, error: float, dt: float = 0.1) -> float:
        """PID çıktısını hesapla"""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def reset(self):
        """PID değerlerini sıfırla"""
        self.integral = 0
        self.prev_error = 0


class Servo:
    """Servo motor sınıfı"""
    def __init__(self, name: str, port: int, initial_position: float = 0.0):
        self.name = name
        self.port = port
        self.position = initial_position  # 0.0 - 1.0 arası
        self.min_angle = 0.0
        self.max_angle = 180.0
        self.history = [initial_position]
    
    def set_position(self, position: float):
        """Servo pozisyonunu ayarla (0.0 - 1.0)"""
        self.position = max(0.0, min(1.0, position))
        self.history.append(self.position)
    
    def get_angle(self) -> float:
        """Mevcut açıyı derece olarak döndür"""
        return self.min_angle + (self.max_angle - self.min_angle) * self.position
    
    def __str__(self):
        return f"Servo '{self.name}' (Port {self.port}): {self.position:.2f} ({self.get_angle():.1f}°)"


class Motor:
    """Ek motor sınıfı (arm, intake vb.)"""
    def __init__(self, name: str, port: int, gear_ratio: float = 1.0):
        self.name = name
        self.port = port
        self.gear_ratio = gear_ratio
        self.power = 0.0
        self.position = 0  # Encoder pozisyonu
        self.target_position = 0
        self.history = []
    
    def set_power(self, power: float):
        """Motor gücünü ayarla (-1.0 ile 1.0 arası)"""
        self.power = max(-1.0, min(1.0, power))
    
    def set_target_position(self, ticks: int):
        """Hedef encoder pozisyonunu ayarla"""
        self.target_position = ticks
    
    def move_to_position(self, ticks: int, speed: float = 0.5):
        """Belirtilen pozisyona git"""
        self.target_position = ticks
        self.power = speed if ticks > self.position else -speed
        # Simülasyon için pozisyonu güncelle
        self.position = ticks
        self.history.append(self.position)
    
    def __str__(self):
        return f"Motor '{self.name}' (Port {self.port}): Güç={self.power:.2f}, Pozisyon={self.position}"


class Robot:
    """FTC Robot sınıfı - konum, yönelim, hareket kontrolü, servo ve motorlar"""
    def __init__(self, x: float = 0, y: float = 0, angle: float = 0,
                 wheel_radius: float = 5.0, length: float = 45.0, width: float = 45.0,
                 kp: float = 1.0, ki: float = 0.0, kd: float = 0.1):
        self.x = x  # cm
        self.y = y  # cm
        self.angle = angle  # derece (0° = yukarı, saat yönü)
        self.wheel_radius = wheel_radius  # cm
        self.length = length  # cm
        self.width = width  # cm
        
        # PID kontrolcüleri
        self.pid_distance = PIDController(kp, ki, kd)
        self.pid_angle = PIDController(kp, ki, kd)
        
        # Hareket geçmişi
        self.path_x = [x]
        self.path_y = [y]
        self.path_angles = [angle]
        
        # Servo ve motorlar
        self.servos: Dict[str, Servo] = {}
        self.motors: Dict[str, Motor] = {}
    
    def add_servo(self, name: str, port: int, initial_position: float = 0.0):
        """Robot'a servo ekle"""
        self.servos[name] = Servo(name, port, initial_position)
        print(f"  ✓ Servo eklendi: {name} (Port {port})")
    
    def add_motor(self, name: str, port: int, gear_ratio: float = 1.0):
        """Robot'a ek motor ekle"""
        self.motors[name] = Motor(name, port, gear_ratio)
        print(f"  ✓ Motor eklendi: {name} (Port {port}, Oran: {gear_ratio})")
    
    def control_servo(self, name: str, position: float):
        """Servo'yu kontrol et"""
        if name in self.servos:
            self.servos[name].set_position(position)
    
    def control_motor(self, name: str, action: str, value: float):
        """Motor'u kontrol et"""
        if name in self.motors:
            if action == 'power':
                self.motors[name].set_power(value)
            elif action == 'position':
                self.motors[name].move_to_position(int(value))
    
    def move_forward(self, distance: float, steps: int = 50):
        """İleri hareket - PID kontrolü ile"""
        self.pid_distance.reset()
        
        # Hedef pozisyon
        target_x = self.x + distance * math.sin(math.radians(self.angle))
        target_y = self.y + distance * math.cos(math.radians(self.angle))
        
        # Adım adım hareket
        for step in range(steps):
            # Mevcut hata
            dx = target_x - self.x
            dy = target_y - self.y
            error = math.sqrt(dx**2 + dy**2)
            
            if error < 0.1:  # Hedefe ulaşıldı
                break
            
            # PID çıktısı
            control = self.pid_distance.calculate(error)
            
            # Hareket miktarı (maksimum adım boyutu ile sınırla)
            step_size = min(control, distance / steps)
            
            # Konum güncelle
            self.x += step_size * math.sin(math.radians(self.angle))
            self.y += step_size * math.cos(math.radians(self.angle))
            
            # Yolu kaydet
            self.path_x.append(self.x)
            self.path_y.append(self.y)
            self.path_angles.append(self.angle)
        
        # Son pozisyonu tam hedefe ayarla
        self.x = target_x
        self.y = target_y
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        self.path_angles.append(self.angle)
    
    def turn(self, degrees: float, steps: int = 30):
        """Dönüş hareketi - PID kontrolü ile"""
        self.pid_angle.reset()
        
        target_angle = self.angle + degrees
        
        # Adım adım dönüş
        for step in range(steps):
            error = target_angle - self.angle
            
            if abs(error) < 0.5:  # Hedefe ulaşıldı
                break
            
            # PID çıktısı
            control = self.pid_angle.calculate(error)
            
            # Açı güncelle (maksimum adım ile sınırla)
            turn_step = min(abs(control), abs(degrees) / steps) * (1 if error > 0 else -1)
            self.angle += turn_step
            
            # Yolu kaydet (aynı konumda ama farklı açı)
            self.path_x.append(self.x)
            self.path_y.append(self.y)
            self.path_angles.append(self.angle)
        
        # Son açıyı tam hedefe ayarla
        self.angle = target_angle
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        self.path_angles.append(self.angle)
    
    def strafe(self, distance: float, direction: str = 'right', steps: int = 50):
        """Yanal hareket (mecanum wheel için)"""
        self.pid_distance.reset()
        
        # Yön belirle
        strafe_angle = self.angle + (90 if direction == 'right' else -90)
        
        # Hedef pozisyon
        target_x = self.x + distance * math.sin(math.radians(strafe_angle))
        target_y = self.y + distance * math.cos(math.radians(strafe_angle))
        
        # Adım adım hareket
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
    
    def get_state(self) -> Dict:
        """Robot durumunu döndür"""
        return {
            'x': self.x,
            'y': self.y,
            'angle': self.angle,
            'wheel_radius': self.wheel_radius,
            'length': self.length,
            'width': self.width,
            'servos': {name: servo.position for name, servo in self.servos.items()},
            'motors': {name: motor.position for name, motor in self.motors.items()}
        }
    
    def reset_path(self):
        """Yol geçmişini sıfırla"""
        self.path_x = [self.x]
        self.path_y = [self.y]
        self.path_angles = [self.angle]


class CommandBlock:
    """Komut bloğu - çeşitli robot hareketleri ve kontroller"""
    def __init__(self, command_type: str, value: float = 0, name: str = "", extra: Dict = None):
        self.type = command_type
        self.value = value
        self.name = name  # Servo/motor adı için
        self.extra = extra or {}
    
    def execute(self, robot: Robot):
        """Komutu robotta çalıştır"""
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
        elif self.type == 'wait':
            pass  # Bekleme simülasyonda pasif
    
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
        elif self.type == 'wait':
            return f"Bekle: {self.value} ms"
        return "Bilinmeyen Komut"


class FTCField:
    """FTC DECODE™ 2025-2026 Alan Bilgileri"""
    def __init__(self):
        # Alan boyutları (cm cinsinden)
        self.width = 365.76  # 12 feet = 365.76 cm
        self.length = 365.76  # 12 feet = 365.76 cm
        
        # Önemli noktalar (DECODE oyunu için)
        self.baskets = [
            {'name': 'Kırmızı Basket', 'x': 60, 'y': 60, 'color': 'red'},
            {'name': 'Mavi Basket', 'x': 305, 'y': 305, 'color': 'blue'},
        ]
        
        self.specimen_bars = [
            {'name': 'Kırmızı Bar', 'x': 30, 'y': 182.88, 'width': 60, 'height': 10, 'color': 'darkred'},
            {'name': 'Mavi Bar', 'x': 275, 'y': 182.88, 'width': 60, 'height': 10, 'color': 'darkblue'},
        ]
        
        self.observation_zones = [
            {'name': 'Kırmızı Gözlem', 'x': 0, 'y': 0, 'width': 60, 'height': 60, 'color': 'lightcoral'},
            {'name': 'Mavi Gözlem', 'x': 305, 'y': 305, 'width': 60, 'height': 60, 'color': 'lightblue'},
        ]
        
        self.submersibles = [
            {'x': 182.88, 'y': 60, 'radius': 25},
            {'x': 182.88, 'y': 305, 'radius': 25},
        ]
        
        # AprilTag konumları
        self.apriltags = [
            {'id': 11, 'x': 0, 'y': 182.88},
            {'id': 12, 'x': 182.88, 'y': 0},
            {'id': 13, 'x': 365.76, 'y': 182.88},
            {'id': 14, 'x': 182.88, 'y': 365.76},
        ]
    
    def draw_field(self, ax):
        """Alan elemanlarını çiz"""
        # Alan sınırları
        field_border = Rectangle((0, 0), self.width, self.length, 
                                 fill=False, edgecolor='black', linewidth=3)
        ax.add_patch(field_border)
        
        # Merkez çizgi
        ax.plot([self.width/2, self.width/2], [0, self.length], 'k--', linewidth=2, alpha=0.3)
        ax.plot([0, self.width], [self.length/2, self.length/2], 'k--', linewidth=2, alpha=0.3)
        
        # Basketler
        for basket in self.baskets:
            circle = Circle((basket['x'], basket['y']), 20, 
                          facecolor=basket['color'], edgecolor='black', alpha=0.5)
            ax.add_patch(circle)
            ax.text(basket['x'], basket['y'], basket['name'], 
                   ha='center', va='center', fontsize=8, color='white', weight='bold')
        
        # Specimen barlar
        for bar in self.specimen_bars:
            rect = Rectangle((bar['x'], bar['y']), bar['width'], bar['height'],
                           facecolor=bar['color'], edgecolor='black', alpha=0.6)
            ax.add_patch(rect)
        
        # Gözlem bölgeleri
        for zone in self.observation_zones:
            rect = Rectangle((zone['x'], zone['y']), zone['width'], zone['height'],
                           facecolor=zone['color'], edgecolor='black', alpha=0.3, linestyle='--')
            ax.add_patch(rect)
        
        # Submersible'lar
        for sub in self.submersibles:
            circle = Circle((sub['x'], sub['y']), sub['radius'], 
                          facecolor='yellow', edgecolor='black', alpha=0.6)
            ax.add_patch(circle)
        
        # AprilTag'ler
        for tag in self.apriltags:
            ax.plot(tag['x'], tag['y'], 'gs', markersize=10, markeredgecolor='black')
            ax.text(tag['x'], tag['y']-15, f"Tag {tag['id']}", 
                   ha='center', fontsize=7, color='green', weight='bold')
        
        # Izgara
        ax.set_xlim(-20, self.width + 20)
        ax.set_ylim(-20, self.length + 20)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        ax.set_xlabel('X (cm)', fontsize=11)
        ax.set_ylabel('Y (cm)', fontsize=11)


class Simulation:
    """Simülasyon yöneticisi"""
    def __init__(self, robot: Robot):
        self.robot = robot
        self.commands: List[CommandBlock] = []
        self.field = FTCField()
    
    def add_command(self, cmd_type: str, value: float = 0, name: str = "", extra: Dict = None):
        """Komut ekle"""
        self.commands.append(CommandBlock(cmd_type, value, name, extra))
        print(f"✓ Komut eklendi: {self.commands[-1]}")
    
    def remove_command(self, index: int):
        """Komut sil"""
        if 0 <= index < len(self.commands):
            removed = self.commands.pop(index)
            print(f"✓ Komut silindi: {removed}")
        else:
            print("✗ Geçersiz komut indeksi!")
    
    def list_commands(self):
        """Komutları listele"""
        if not self.commands:
            print("Henüz komut eklenmedi.")
            return
        
        print("\n=== Komut Dizisi ===")
        for i, cmd in enumerate(self.commands):
            print(f"{i+1}. {cmd}")
        print("=" * 40)
    
    def run(self):
        """Simülasyonu çalıştır"""
        print("\n🤖 Simülasyon başlatılıyor...")
        
        # Robot konumunu sıfırla
        initial_x, initial_y, initial_angle = self.robot.x, self.robot.y, self.robot.angle
        self.robot.reset_path()
        
        # Komutları çalıştır
        for i, cmd in enumerate(self.commands):
            print(f"  {i+1}. {cmd} çalıştırılıyor...")
            cmd.execute(self.robot)
        
        # Sonuçları raporla
        print("\n✓ Simülasyon tamamlandı!")
        print(f"\n📍 Başlangıç: X={initial_x:.2f} cm, Y={initial_y:.2f} cm, Açı={initial_angle:.2f}°")
        print(f"📍 Bitiş: X={self.robot.x:.2f} cm, Y={self.robot.y:.2f} cm, Açı={self.robot.angle:.2f}°")
        print(f"📏 Toplam adım sayısı: {len(self.robot.path_x)}")
        
        # Servo ve motor durumları
        if self.robot.servos:
            print("\n🔧 Servo Durumları:")
            for name, servo in self.robot.servos.items():
                print(f"  {servo}")
        
        if self.robot.motors:
            print("\n⚙️ Motor Durumları:")
            for name, motor in self.robot.motors.items():
                print(f"  {motor}")
    
    def visualize(self, show_field: bool = True):
        """Simülasyonu görselleştir"""
        fig, ax = plt.subplots(figsize=(14, 14))
        
        # Alan çiz
        if show_field:
            self.field.draw_field(ax)
            ax.set_title('AutoTM - FTC DECODE™ 2025-2026 Robot Simülasyonu\nMade by Error 24501', 
                        fontsize=16, fontweight='bold', pad=20)
        else:
            ax.set_title('AutoTM - FTC Robot Simülasyonu\nMade by Error 24501', 
                        fontsize=16, fontweight='bold', pad=20)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
        
        # Yolu çiz
        ax.plot(self.robot.path_x, self.robot.path_y, 'b-', 
               linewidth=3, label='Robot Yolu', alpha=0.7)
        
        # Başlangıç ve bitiş noktaları
        ax.plot(self.robot.path_x[0], self.robot.path_y[0], 'go', 
               markersize=20, label='Başlangıç', markeredgecolor='black', markeredgewidth=2)
        ax.plot(self.robot.path_x[-1], self.robot.path_y[-1], 'ro', 
               markersize=20, label='Bitiş', markeredgecolor='black', markeredgewidth=2)
        
        # Robot yönünü göster (her 10 adımda bir)
        step_interval = max(1, len(self.robot.path_x) // 25)
        for i in range(0, len(self.robot.path_x), step_interval):
            x, y, angle = self.robot.path_x[i], self.robot.path_y[i], self.robot.path_angles[i]
            dx = 15 * math.sin(math.radians(angle))
            dy = 15 * math.cos(math.radians(angle))
            ax.arrow(x, y, dx, dy, head_width=5, head_length=5, 
                    fc='darkred', ec='darkred', alpha=0.6, linewidth=2)
        
        # Son robot konumu (dikdörtgen olarak)
        final_angle = self.robot.path_angles[-1]
        rect = Rectangle(
            (self.robot.x - self.robot.width/2, self.robot.y - self.robot.length/2),
            self.robot.width, self.robot.length,
            angle=final_angle, facecolor='orange', alpha=0.7, 
            edgecolor='black', linewidth=3
        )
        ax.add_patch(rect)
        
        # Legend
        ax.legend(loc='upper right', fontsize=11, framealpha=0.9)
        
        plt.tight_layout()
        plt.show()
    
    def save_commands(self, filename: str = "commands.json"):
        """Komutları kaydet"""
        data = {
            'commands': [
                {
                    'type': cmd.type, 
                    'value': cmd.value, 
                    'name': cmd.name,
                    'extra': cmd.extra
                } for cmd in self.commands
            ],
            'robot_state': self.robot.get_state()
        }
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"✓ Komutlar '{filename}' dosyasına kaydedildi.")
    
    def load_commands(self, filename: str = "commands.json"):
        """Komutları yükle"""
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self.commands = [
                CommandBlock(cmd['type'], cmd['value'], cmd.get('name', ''), cmd.get('extra', {}))
                for cmd in data['commands']
            ]
            print(f"✓ {len(self.commands)} komut '{filename}' dosyasından yüklendi.")
            self.list_commands()
        except FileNotFoundError:
            print(f"✗ '{filename}' dosyası bulunamadı!")
        except Exception as e:
            print(f"✗ Dosya yüklenirken hata: {e}")
    
    def generate_java_code(self) -> str:
        """FTC Java LinearOpMode kodu üret"""
        code = """// AutoTM - FTC Otonom Java Kodu (LinearOpMode)
// Made by Error 24501
// Bu kod Python simülasyonundan otomatik üretilmiştir
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
            for name, servo in self.robot.servos.items():
                code += f"    private Servo {name}Servo;\n"
            code += "\n"
        
        # Ek motor tanımlamaları
        if self.robot.motors:
            code += "    // Ek motorlar\n"
            for name, motor in self.robot.motors.items():
                code += f"    private DcMotor {name}Motor;\n"
            code += "\n"
        
        # PID parametreleri
        code += f"    // PID parametreleri\n"
        code += f"    private static final double Kp = {self.robot.pid_distance.kp};\n"
        code += f"    private static final double Ki = {self.robot.pid_distance.ki};\n"
        code += f"    private static final double Kd = {self.robot.pid_distance.kd};\n"
        code += f"    private static final double WHEEL_RADIUS_CM = {self.robot.wheel_radius};\n"
        code += f"    private static final int TICKS_PER_REV = 1120; // Encoder ticks per revolution\n"
        code += """    
    @Override
    public void runOpMode() {
        // Sürüş motorlarını başlat
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        
        // Motor yönlerini ayarla
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        
"""
        
        # Servo başlatma
        if self.robot.servos:
            code += "        // Servo'ları başlat\n"
            for name, servo in self.robot.servos.items():
                code += f"        {name}Servo = hardwareMap.get(Servo.class, \"{name}\");\n"
            code += "\n"
        
        # Ek motor başlatma
        if self.robot.motors:
            code += "        // Ek motorları başlat\n"
            for name, motor in self.robot.motors.items():
                code += f"        {name}Motor = hardwareMap.get(DcMotor.class, \"{name}\");\n"
                code += f"        {name}Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);\n"
                code += f"        {name}Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);\n"
            code += "\n"
        
        code += """        // Encoder'ları sıfırla
        resetEncoders();
        
        telemetry.addData("Status", "Hazır - AutoTM by Error 24501");
        telemetry.addData("Game", "DECODE 2025-2026");
        telemetry.addData("Robot", "Başlangıç X=" + """ + f"{self.robot.path_x[0]:.1f}" + """ + ", Y=" + """ + f"{self.robot.path_y[0]:.1f}" + """);
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // ===== OTONOM KOMUTLAR =====
"""
        
        # Komutları Java koduna çevir
        for i, cmd in enumerate(self.commands):
            code += f"            // {i+1}. Komut: {cmd}\n"
            
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
            elif cmd.type == 'wait':
                code += f"            sleep({int(cmd.value)});\n"
            
            code += "\n"
        
        code += """            
            telemetry.addData("Status", "Otonom Tamamlandı!");
            telemetry.addData("Team", "Error 24501 - AutoTM");
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
            telemetry.addData("İleri", "Hedef: %d cm, Mevcut: %d ticks", 
                (int)distanceCm, leftFrontMotor.getCurrentPosition());
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
        // Robot genişliğine göre dönüş hesabı (ayarlanmalı)
        double wheelBase = 35.0; // cm - robot tekerlekleri arası mesafe
        double arcLength = Math.PI * wheelBase * degrees / 180.0;
        int turnTicks = cmToTicks(arcLength);
        
        resetEncoders();
        
        // Sol motorlar ileri, sağ motorlar geri (veya tersi)
        if (degrees > 0) { // Sağa dönüş
            setTargetPosition(turnTicks, -turnTicks, turnTicks, -turnTicks);
        } else { // Sola dönüş
            setTargetPosition(-turnTicks, turnTicks, -turnTicks, turnTicks);
        }
        
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(0.3, 0.3, 0.3, 0.3);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Dönüş", "Hedef: %.1f°", degrees);
            telemetry.update();
        }
        
        stopMotors();
    }
    
    private void strafeRight(double distanceCm) {
        int targetTicks = cmToTicks(distanceCm);
        
        resetEncoders();
        // Mecanum wheel strafe: LF(-), RF(+), LB(+), RB(-)
        setTargetPosition(-targetTicks, targetTicks, targetTicks, -targetTicks);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        setPower(0.5, 0.5, 0.5, 0.5);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Sağa Kayma", "%d cm", (int)distanceCm);
            telemetry.update();
        }
        
        stopMotors();
    }
    
    private void strafeLeft(double distanceCm) {
        int targetTicks = cmToTicks(distanceCm);
        
        resetEncoders();
        // Mecanum wheel strafe: LF(+), RF(-), LB(-), RB(+)
        setTargetPosition(targetTicks, -targetTicks, -targetTicks, targetTicks);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        setPower(0.5, 0.5, 0.5, 0.5);
        
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Sola Kayma", "%d cm", (int)distanceCm);
            telemetry.update();
        }
        
        stopMotors();
    }
    
    private void moveMotorToPosition(DcMotor motor, int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);
        
        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Motor Pozisyon", "%d / %d", 
                motor.getCurrentPosition(), targetPosition);
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
    
    def export_java(self, filename: str = "AutoTM_DECODE.java"):
        """Java kodunu dosyaya kaydet"""
        code = self.generate_java_code()
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(code)
        print(f"✓ Java kodu '{filename}' dosyasına kaydedildi.")
        print(f"  Made by Error 24501 - AutoTM")


def print_header():
    """Başlık yazdır"""
    print("=" * 70)
    print("🤖 AutoTM - FTC OTONOM ROBOT SİMÜLASYONU")
    print("   Made by Error 24501")
    print("=" * 70)
    print("   DECODE™ 2025-2026 Sezon Desteği")
    print("   Servo Kontrol | Ek Motorlar | Mecanum Wheel | PID Kontrol")
    print("=" * 70)


def setup_robot():
    """Robot kurulumu"""
    print("\n📝 Robot Parametreleri (Enter: varsayılan değer)")
    
    try:
        x = float(input("Başlangıç X (cm) [60]: ") or 60)
        y = float(input("Başlangıç Y (cm) [60]: ") or 60)
        angle = float(input("Başlangıç açı (derece, 0=yukarı) [0]: ") or 0)
        kp = float(input("PID Kp [1.0]: ") or 1.0)
        ki = float(input("PID Ki [0.0]: ") or 0.0)
        kd = float(input("PID Kd [0.1]: ") or 0.1)
    except ValueError:
        print("✗ Geçersiz giriş! Varsayılan değerler kullanılıyor.")
        x, y, angle, kp, ki, kd = 60, 60, 0, 1.0, 0.0, 0.1
    
    robot = Robot(x, y, angle, kp=kp, ki=ki, kd=kd)
    
    print(f"\n✓ Robot oluşturuldu: X={x}, Y={y}, Açı={angle}°")
    print(f"  PID: Kp={kp}, Ki={ki}, Kd={kd}")
    
    # Servo ve motor ekleme
    print("\n🔧 Servo/Motor Eklemek İster Misiniz? (e/h)")
    if input().lower() == 'e':
        setup_components(robot)
    
    return robot


def setup_components(robot: Robot):
    """Servo ve motor kurulumu"""
    print("\n=== Komponent Kurulumu ===")
    
    # Servo ekleme
    print("\nKaç servo eklemek istersiniz? [0-4]: ", end='')
    try:
        servo_count = int(input() or 0)
        for i in range(servo_count):
            name = input(f"  Servo {i+1} adı [servo{i+1}]: ") or f"servo{i+1}"
            port = int(input(f"  Servo {i+1} port [0-5]: ") or i)
            pos = float(input(f"  Servo {i+1} başlangıç pozisyonu [0.0-1.0]: ") or 0.0)
            robot.add_servo(name, port, pos)
    except ValueError:
        print("✗ Geçersiz giriş!")
    
    # Motor ekleme
    print("\nKaç ek motor eklemek istersiniz? (arm, intake, lift vb.) [0-4]: ", end='')
    try:
        motor_count = int(input() or 0)
        for i in range(motor_count):
            name = input(f"  Motor {i+1} adı [motor{i+1}]: ") or f"motor{i+1}"
            port = int(input(f"  Motor {i+1} port [0-7]: ") or i)
            ratio = float(input(f"  Motor {i+1} gear ratio [1.0]: ") or 1.0)
            robot.add_motor(name, port, ratio)
    except ValueError:
        print("✗ Geçersiz giriş!")


def show_menu():
    """Ana menüyü göster"""
    print("\n" + "=" * 70)
    print("📋 ANA MENÜ:")
    print("-" * 70)
    print("  HAREKET KOMUTLARI:")
    print("    1. İleri hareket          2. Geri hareket")
    print("    3. Sağa dön              4. Sola dön")
    print("    5. Sağa kayma (strafe)   6. Sola kayma (strafe)")
    print("\n  SERVO/MOTOR KOMUTLARI:")
    print("    7. Servo kontrol         8. Motor güç ayarla")
    print("    9. Motor pozisyon ayarla 10. Bekleme ekle")
    print("\n  SİMÜLASYON:")
    print("    11. Komutları listele    12. Komut sil")
    print("    13. Simülasyonu çalıştır 14. Görselleştir (Alan ile)")
    print("    15. Görselleştir (Alan olmadan)")
    print("\n  JAVA & KAYIT:")
    print("    16. Java kodu üret       17. Komutları kaydet")
    print("    18. Komutları yükle")
    print("\n  DİĞER:")
    print("    19. Robot durumunu gör   20. Yeni robot oluştur")
    print("    0. Çıkış")
    print("=" * 70)


def main():
    """Ana program"""
    print_header()
    
    robot = setup_robot()
    sim = Simulation(robot)
    
    # Ana döngü
    while True:
        show_menu()
        choice = input("\n👉 Seçiminiz: ").strip()
        
        try:
            if choice == '1':  # İleri
                value = float(input("İleri hareket mesafesi (cm): "))
                sim.add_command('forward', value)
            
            elif choice == '2':  # Geri
                value = float(input("Geri hareket mesafesi (cm): "))
                sim.add_command('backward', value)
            
            elif choice == '3':  # Sağa dön
                value = float(input("Sağa dönüş açısı (derece): "))
                sim.add_command('right', value)
            
            elif choice == '4':  # Sola dön
                value = float(input("Sola dönüş açısı (derece): "))
                sim.add_command('left', value)
            
            elif choice == '5':  # Sağa kayma
                value = float(input("Sağa kayma mesafesi (cm): "))
                sim.add_command('strafe_right', value)
            
            elif choice == '6':  # Sola kayma
                value = float(input("Sola kayma mesafesi (cm): "))
                sim.add_command('strafe_left', value)
            
            elif choice == '7':  # Servo kontrol
                if not robot.servos:
                    print("✗ Robot'ta servo yok!")
                else:
                    print("Mevcut servo'lar:", ", ".join(robot.servos.keys()))
                    name = input("Servo adı: ")
                    if name in robot.servos:
                        pos = float(input("Pozisyon (0.0-1.0): "))
                        sim.add_command('servo', pos, name)
                    else:
                        print("✗ Servo bulunamadı!")
            
            elif choice == '8':  # Motor güç
                if not robot.motors:
                    print("✗ Robot'ta ek motor yok!")
                else:
                    print("Mevcut motorlar:", ", ".join(robot.motors.keys()))
                    name = input("Motor adı: ")
                    if name in robot.motors:
                        power = float(input("Güç (-1.0 ile 1.0): "))
                        sim.add_command('motor_power', power, name)
                    else:
                        print("✗ Motor bulunamadı!")
            
            elif choice == '9':  # Motor pozisyon
                if not robot.motors:
                    print("✗ Robot'ta ek motor yok!")
                else:
                    print("Mevcut motorlar:", ", ".join(robot.motors.keys()))
                    name = input("Motor adı: ")
                    if name in robot.motors:
                        pos = int(input("Hedef pozisyon (encoder ticks): "))
                        sim.add_command('motor_position', pos, name)
                    else:
                        print("✗ Motor bulunamadı!")
            
            elif choice == '10':  # Bekleme
                value = float(input("Bekleme süresi (ms): "))
                sim.add_command('wait', value)
            
            elif choice == '11':  # Komutları listele
                sim.list_commands()
            
            elif choice == '12':  # Komut sil
                sim.list_commands()
                index = int(input("Silinecek komut numarası: ")) - 1
                sim.remove_command(index)
            
            elif choice == '13':  # Simülasyon çalıştır
                if not sim.commands:
                    print("✗ Önce komut eklemelisiniz!")
                else:
                    sim.run()
            
            elif choice == '14':  # Görselleştir (Alan ile)
                if not sim.commands:
                    print("✗ Önce komut ekleyip simülasyonu çalıştırmalısınız!")
                else:
                    sim.run()
                    sim.visualize(show_field=True)
            
            elif choice == '15':  # Görselleştir (Alan olmadan)
                if not sim.commands:
                    print("✗ Önce komut ekleyip simülasyonu çalıştırmalısınız!")
                else:
                    sim.run()
                    sim.visualize(show_field=False)
            
            elif choice == '16':  # Java kodu
                if not sim.commands:
                    print("✗ Önce komut eklemelisiniz!")
                else:
                    print("\n" + "=" * 70)
                    print(sim.generate_java_code())
                    print("=" * 70)
                    export = input("\nDosyaya kaydetmek ister misiniz? (e/h): ").lower()
                    if export == 'e':
                        filename = input("Dosya adı [AutoTM_DECODE.java]: ") or "AutoTM_DECODE.java"
                        sim.export_java(filename)
            
            elif choice == '17':  # Komutları kaydet
                filename = input("Dosya adı [commands.json]: ") or "commands.json"
                sim.save_commands(filename)
            
            elif choice == '18':  # Komutları yükle
                filename = input("Dosya adı [commands.json]: ") or "commands.json"
                sim.load_commands(filename)
            
            elif choice == '19':  # Robot durumu
                print("\n" + "=" * 70)
                print("🤖 ROBOT DURUMU")
                print("-" * 70)
                state = robot.get_state()
                print(f"Pozisyon: X={state['x']:.2f} cm, Y={state['y']:.2f} cm")
                print(f"Açı: {state['angle']:.2f}°")
                print(f"Boyutlar: {state['length']}x{state['width']} cm")
                
                if robot.servos:
                    print("\n🔧 Servo'lar:")
                    for name, servo in robot.servos.items():
                        print(f"  {servo}")
                
                if robot.motors:
                    print("\n⚙️ Motorlar:")
                    for name, motor in robot.motors.items():
                        print(f"  {motor}")
                print("=" * 70)
            
            elif choice == '20':  # Yeni robot
                print("\n🔄 Yeni robot oluşturuluyor...")
                robot = setup_robot()
                sim = Simulation(robot)
            
            elif choice == '0':  # Çıkış
                print("\n👋 Programdan çıkılıyor...")
                print("=" * 70)
                print("AutoTM by Error 24501")
                print("DECODE™ ile başarılar! 🏆")
                print("=" * 70)
                break
            
            else:
                print("✗ Geçersiz seçim!")
        
        except ValueError:
            print("✗ Geçersiz değer girdiniz!")
        except Exception as e:
            print(f"✗ Hata oluştu: {e}")


if __name__ == "__main__":
    main()
