import sys
import os
import time
from dotenv import load_dotenv
from PyQt5.QtWidgets import (QApplication, QLabel, QPushButton, QVBoxLayout, 
                               QHBoxLayout, QWidget, QGraphicsView, QGraphicsScene, 
                               QGraphicsRectItem, QGraphicsLineItem, QGraphicsPolygonItem, QGroupBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QPointF
from PyQt5.QtGui import QBrush, QPen, QColor, QPolygonF
from pymavlink import mavutil

load_dotenv()

# Harita alanı için özelleştirilmiş QGraphicsView sınıfı
"""
    Raporda belirtilen ekstra özellikler arasından:
        1. Fare imcleci tabanlı yakınlaştırma ve uzaklaştırma -> wheelEvent fonksiyonu ile
        2. Fareyle haritayı kaydırma -> dragMode parametresi -> ScrollHandDrag
    yapıldı.
"""

class CustomGraphicsView(QGraphicsView):
    
    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        
    def wheelEvent(self, event):
        zoom_in_factor = 1.15
        zoom_out_factor = 1.0 / zoom_in_factor
        
        if event.angleDelta().y() > 0:
            self.scale(zoom_in_factor, zoom_in_factor)
        else:
            self.scale(zoom_out_factor, zoom_out_factor)


# MAVLink komutlarını dinleyecek olan thread sınıfı.
"""
    Kesmeler:
        1. Yükseklik kesmesi
        2. Durum kesmesi
        3. GPS kesmesi
        4. Airspeed kesmesi
        5. mode kesmesi
        6. Heading kesmesi
"""

class MavLinkListenerThread(QThread):
    altitude_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str)
    gps_signal = pyqtSignal(float, float)
    airspeed_signal = pyqtSignal(str)
    mode_signal = pyqtSignal(str)
    heading_signal = pyqtSignal(float)
    
    def __init__(self):
        super().__init__()
        self.running = True
        self.is_listening = False
        self.master = None
    
    
    def start_listening(self):
        self.is_listening = True
        self.status_signal.emit("Durum: Bağlantı başlatılıyor...")
        
    def stop_listening(self):
        self.is_listening = False
        if self.master:
            try:
                self.master.close()
            except:
                pass
            self.master = None
        self.status_signal.emit("Durum: Bağlantı kesildi. (Veriler güncel değildir)")
        
    def run(self):
        
        # Bağlantı adresini .env dosyasından çekiyoruz
        connection_string = os.getenv("MAVLINK_CONNECTION", "udpin:127.0.0.1:14550")
        
        while self.running:
            if not self.is_listening:
                time.sleep(0.1)
                continue
            
            print(f"SITL bağlantısı denetleniyor ({connection_string})...")
            try:
                master = mavutil.mavlink_connection(connection_string)
                self.status_signal.emit("Durum: Bağlantı bekleniyor...")
                
                last_message_time = time.time()
                
                while self.is_listening and self.running:
                    msg = master.recv_match(blocking=True, timeout=2.0)
                    
                    if not msg:
                        if time.time() - last_message_time > 3.0:
                            self.status_signal.emit("Durum: Bağlantı koptu, en son bu veriler vardı.")
                            break
                        continue
                    
                    last_message_time = time.time()
                    msg_type = msg.get_type()
                    
                    if msg_type == "GLOBAL_POSITION_INT":
                        alt = msg.relative_alt / 1000.0  # Metreye çevrilir
                        self.altitude_signal.emit(f"İrtifa: {alt:.2f} m")
                        
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        self.gps_signal.emit(lat, lon)

                    elif msg_type == "VFR_HUD":
                        airspeed = msg.airspeed
                        heading = msg.heading
                        self.airspeed_signal.emit(f"Hız (Airspeed): {airspeed:.2f} m/s")
                        self.heading_signal.emit(float(heading))

                    elif msg_type == "HEARTBEAT":
                        self.status_signal.emit("Durum: Bağlantı Kuruldu (Aktif)")
                        try:
                            mode = mavutil.mode_string_v10(msg)
                            self.mode_signal.emit(f"Uçuş Modu: {mode}")
                        except:
                            pass

            except Exception as e:
                self.status_signal.emit(f"Hata: Bağlantı kurulamadı, yeniden deneniyor...")
                time.sleep(3)

class GroundControlStation(QWidget):
    def __init__(self):
        super().__init__()
        
        # Harita coğrafi sınırları (Canberra SITL kalkış alanı merkezli)
        self.min_lat = -35.3800
        self.max_lat = -35.3400
        self.min_lon = 149.1400
        self.max_lon = 149.1900
        
        # İlk bağlantı kurulduğunda uçağı merkeze almak için kullanılır
        self.is_first_fix = True
        
        
        self.mavlink_thread = MavLinkListenerThread()
        
        self.init_ui()

        
        self.mavlink_thread.altitude_signal.connect(self.update_altitude_label)
        self.mavlink_thread.status_signal.connect(self.update_status_label)
        self.mavlink_thread.airspeed_signal.connect(self.update_airspeed_label)
        self.mavlink_thread.gps_signal.connect(self.update_drone_data)
        self.mavlink_thread.mode_signal.connect(self.update_mode_label)
        self.mavlink_thread.heading_signal.connect(self.update_drone_heading)
        self.mavlink_thread.start()
        
        
        
    def init_ui(self):
        self.setWindowTitle("İTU ATA 27 - Ground Station")
        self.resize(950, 600)
        
        main_layout = QHBoxLayout(self)
        
        # --- SOL TARAF: Harita Alanı ---
        map_layout = QVBoxLayout()
        scene_size = 450
        self.scene = QGraphicsScene(0, 0, scene_size, scene_size)
        
        # Özel CustomGraphicsView kullanımı (Zoom ve Panning özellikli)
        self.view = CustomGraphicsView(self.scene, self)
        self.view.setMinimumSize(scene_size, scene_size)
        
        # 1. Siyah harita arkaplanı
        map_bg = QGraphicsRectItem(0, 0, scene_size, scene_size)
        map_bg.setBrush(QBrush(QColor(30, 30, 30)))
        map_bg.setPen(QPen(Qt.white, 2))
        self.scene.addItem(map_bg)
        
        # 2. Izgara Çizgileri (Grid)
        grid_step = 45  # 450'ye 45 1/10 olması için
        grid_pen = QPen(QColor(60, 60, 60), 1, Qt.DashLine)
        
        for x in range(0, scene_size + 1, grid_step):
            line = QGraphicsLineItem(x, 0, x, scene_size)
            line.setPen(grid_pen)
            self.scene.addItem(line)
            
        for y in range(0, scene_size + 1, grid_step):
            line = QGraphicsLineItem(0, y, scene_size, y)
            line.setPen(grid_pen)
            self.scene.addItem(line)

        # 3. Uçağın Yönünü Gösteren Üçgen Simge
        # Merkez etrafında dönmesi için üçgen koordinatları
        triangle = QPolygonF([QPointF(0, -10), QPointF(-7, 8), QPointF(7, 8)])
        self.drone_item = QGraphicsPolygonItem(triangle)
        self.drone_item.setBrush(QBrush(QColor(0, 255, 0))) # Yeşil renk
        self.drone_item.setPen(QPen(Qt.black, 1))
        # Üçgenin kendi merkezi etrafında dönmesi için transform origin ayarlanır
        self.drone_item.setTransformOriginPoint(0, 0)
        self.scene.addItem(self.drone_item)
        
        map_layout.addWidget(self.view)
        
        # --- SAĞ TARAF: Kontroller ve Uçuş Bilgileri ---
        right_panel = QGroupBox("Uçuş Bilgileri ve Kontroller", self)
        right_layout = QVBoxLayout()
        
        self.status_label = QLabel("Durum: Başlatılıyor...", self)
        right_layout.addWidget(self.status_label)
        
        # Bağlantı Kontrol Butonları
        self.start_btn = QPushButton("Bağlantıyı Başlat", self)
        self.start_btn.clicked.connect(self.mavlink_thread.start_listening)
        right_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("Bağlantıyı Kes", self)
        self.stop_btn.clicked.connect(self.mavlink_thread.stop_listening)
        right_layout.addWidget(self.stop_btn)
        
        self.mode_label = QLabel("Uçuş Modu: Bekleniyor...", self)
        right_layout.addWidget(self.mode_label)
        
        self.altitude_label = QLabel("İrtifa Bekleniyor...", self)
        right_layout.addWidget(self.altitude_label)
        
        self.airspeed_label = QLabel("Hız Bekleniyor...", self)
        right_layout.addWidget(self.airspeed_label)
        
        self.gps_label = QLabel("Konum Bekleniyor...", self)
        right_layout.addWidget(self.gps_label)
        
        
        # Uçağa Yeniden Merkezleme Düğmesi
        self.recenter_btn = QPushButton("Uçağa Merkezle", self)
        self.recenter_btn.clicked.connect(self.recenter_map)
        right_layout.addWidget(self.recenter_btn)
        
        right_layout.addStretch()
        right_panel.setLayout(right_layout)
        
        main_layout.addLayout(map_layout, stretch=2)
        main_layout.addWidget(right_panel, stretch=1)
        
        self.setLayout(main_layout) #En son main layout yüklenir
        
        
    # Enlem ve boylamı piksellere çevirmek için kullanılan doğrusal dönüştürme yöntemi.
    """
        Çalışma prensibi doğrusal ilişkiye dayanmaktadır. Uçağın SITL ortamındaki minumum ve maksimum enlem, boylam
        değerleri alınır.
        Sahnenin maksimum piksel değerleri de bellidir. (450px)
        Ardından hem x hem de y sahne koordinatları için doğrusal bir ilişki kurularak enlem ve boylamdaki değişimler
        x , y için uygun değişimlerinin hesaplanmasında kullanılır.
    """
    
    def latlon_to_pixel(self, lat, lon):
        scene_width = 450
        scene_height = 450
        
        x = (lon - self.min_lon) / (self.max_lon - self.min_lon) * scene_width
        y = (self.max_lat - lat) / (self.max_lat - self.min_lat) * scene_height
        
        x = max(0, min(x, scene_width)) #Sahneden taşmaması için kullanılır
        y = max(0, min(y, scene_height))
        
        return x, y

    def update_altitude_label(self, alt_text):
        self.altitude_label.setText(alt_text)
        
    def update_status_label(self, status_text):
        self.status_label.setText(status_text)
        
    def update_airspeed_label(self, airspeed_text):
        self.airspeed_label.setText(airspeed_text)
        
    def update_drone_data(self, lat, lon):
        self.gps_label.setText(f"Konum: Lat {lat:.6f}, Lon {lon:.6f}")
        
        x, y = self.latlon_to_pixel(lat, lon)
        self.drone_item.setPos(x, y)
        
        # İlk geçerli konum verisinde otomatik merkezleme
        if self.is_first_fix:
            self.view.centerOn(self.drone_item)
            self.is_first_fix = False
            
    def update_drone_heading(self, heading):
        # Uçuş yönüne (heading) göre üçgen simgeyi döndür
        self.drone_item.setRotation(heading)
    
    def update_mode_label(self, modeText):
        self.mode_label.setText(modeText)
        
    def recenter_map(self):
        # Butona tıklandığında haritayı tekrar uçağın olduğu konuma odaklar
        self.view.centerOn(self.drone_item)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gcs = GroundControlStation()
    gcs.show()
    sys.exit(app.exec_())