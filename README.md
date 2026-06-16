# Precision Landing / VTOL

ROS 2 ve PX4 tabanlı, AprilTag görüntü işleme ile insansız hava aracının (İHA) hedef üzerine hassas iniş yapmasını sağlayan otonom kontrol sistemi.

## Genel Bakış

Sistem iki ana ROS 2 paketi içerir:

- **aruco_tracker** — Kamera görüntüsünden AprilTag tespiti yapar, etiketin ekran merkezine göre sapmasını ve kaplama yüzdesini `/aruco_pose` konusuna yayınlar.
- **offboard_control** — PX4 ile offboard modda haberleşir; ArUco verisini PID kontrolcüsüne besleyerek İHA'yı hedefe yönlendirir ve iniş manevrasını gerçekleştirir.

Gazebo simülasyon modelleri (`Tools/`) ile gerçek donanıma geçmeden önce sanal ortamda test yapılabilir.

## Mimari

```
Gazebo Kamera
      │
      ▼ /camera (sensor_msgs/Image)
aruco_tracker (Python)
      │
      ▼ /aruco_pose (geometry_msgs/Twist)
offboard_control (C++)
      │
      ▼ PX4 uXRCE-DDS
   İHA (arm / disarm / trajectory setpoint)
```

## Gereksinimler

| Bağımlılık | Sürüm |
|---|---|
| ROS 2 | Humble veya üstü |
| PX4 Autopilot | 1.14+ |
| Gazebo | Garden / Harmonic |
| Python | 3.10+ |
| OpenCV | 4.x |
| apriltag (Python) | — |
| cv_bridge | ROS 2 paketinden |
| ros_gz_bridge | Gazebo sürümüyle uyumlu |

## Kurulum

```bash
# Çalışma alanı oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Bu repoyu klonla
git clone https://github.com/oktay-b/precision_landing-vtol.git

# Bağımlılıkları yükle
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Derle
colcon build
source install/setup.bash
```

## Kullanım

### 1. Gazebo Köprüsünü Başlat

```bash
ros2 launch offboard_control gz_bridge_launch.py
```

Bu launch dosyası üç köprü başlatır:
- `/cmd_vel` — kara aracı hız komutları
- `/keyboard/keypress` — klavye girdileri
- `/camera` — Gazebo kamera görüntüsü

### 2. ArUco Tracker Düğümünü Başlat

```bash
ros2 run aruco_tracker aruco_tracker
```

Ekranda `Gazebo Cam` penceresi açılır; tespit edilen AprilTag etrafına yeşil çerçeve çizilir ve merkeze sapma çizgisi gösterilir.

### 3. Offboard Kontrol Düğümünü Başlat

```bash
ros2 run offboard_control offboard_control_node
```

### PID Parametreleri

`src/offboard_control/config/params.yaml` dosyasından ayarlanabilir:

```yaml
offboard_control_node:
  ros__parameters:
    p: 1
    i: 0.1
    d: 0.01
```

## Simülasyon Modelleri

`Tools/simulation/gz/models/` altında bulunan modeller:

| Model | Açıklama |
|---|---|
| `land` | İniş hedef platformu (çeşitli yüzey dokuları) |
| `AprilTagBox` | AprilTag etiketli hedef kutusu |
| `stack_rover` | Kara aracı (LiDAR + kamera donanımlı) |
| `stack_rover_ar` | AprilTag taşıyan kara aracı |
| `axes` | Referans eksen göstergesi |

## Yayınlanan ve Dinlenen Konular

| Konu | Tip | Yön | Açıklama |
|---|---|---|---|
| `/camera` | `sensor_msgs/Image` | Giren | Gazebo kamera görüntüsü |
| `/aruco_pose` | `geometry_msgs/Twist` | Çıkan / Giren | Tag sapma ve durum bilgisi |
| `/keyboard/keypress` | `std_msgs/Int32` | Giren | Klavye komutları |

### `/aruco_pose` Mesaj Alanları

| Alan | Anlam |
|---|---|
| `linear.x` | Yatay eksen sapması (piksel / 100) |
| `linear.y` | Dikey eksen sapması (piksel / 100) |
| `linear.z` | Tag'in görüntüdeki kaplama yüzdesi |
| `angular.x` | Tag bulundu mu? (1.0 = evet) |
| `angular.y` | Tag ID numarası |

## Proje Yapısı

```
precision_land 2/
├── src/
│   ├── aruco_tracker/          # Python — AprilTag tespit paketi
│   │   └── aruco_tracker/
│   │       └── auro_detector.py
│   └── offboard_control/       # C++ — PX4 offboard kontrol paketi
│       ├── config/params.yaml
│       ├── include/
│       ├── launch/
│       └── src/
└── Tools/
    └── simulation/gz/models/   # Gazebo simülasyon modelleri
```

## Lisans

Bu proje açık kaynaklıdır. Lisans dosyası eklenecektir.
