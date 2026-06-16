#!/usr/bin/env python3
import rclpy                                # ROS 2 Python client kütüphanesi
from rclpy.node import Node                 # ROS 2 düğüm sınıfı
from sensor_msgs.msg import Image           # ROS 2'nin görüntü mesaj tipi
from geometry_msgs.msg import Twist         # ROS 2'nin kontrol mesaj tipi
from cv_bridge import CvBridge             # ROS mesajlarını OpenCV formatına çevirmek için
import cv2                                 # OpenCV görüntü işleme kütüphanesi
import apriltag                            # AprilTag tespiti için kullanılan kütüphane
import math
import numpy as np

# AprilTag tespit düğümü
class AuroDetector(Node):
    def __init__(self):
        super().__init__("apriltag_template_detector")  # Düğüm ismi
        self.bridge = CvBridge()                        # ROS Image <-> OpenCV çeviricisi

        # "/camera" konusunu dinle, her yeni görüntü geldiğinde image_callback fonksiyonu çalışacak
        self.subscription = self.create_subscription(
            Image, "/camera", self.image_callback, 10
        )

        # Twist mesajı (hareket bilgisi) yayınlayacağımız publisher
        self.ar_tag_twist = Twist()
        self.publisher = self.create_publisher(Twist, "/aruco_pose", 10)

        # AprilTag dedektörü başlatılıyor
        self.detector = apriltag.Detector()

        # OpenCV penceresi ayarlanıyor
        cv2.namedWindow("Gazebo Cam", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Gazebo Cam", 480, 320)

    def image_callback(self, msg: Image):
        try:
            # ROS Image mesajını OpenCV görüntüsüne çevir
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Griye çevir

            results = self.detector.detect(gray_frame)            # AprilTag'leri tespit et
            frame_height, frame_width, _ = frame.shape            # Görüntü boyutlarını al

            # Görüntü merkezini hesapla
            center_x = frame_width // 2
            center_y = frame_height // 2

            # Görüntü ortasında bir kutu çizilecek (hedef bölge)
            box_size = 200
            box_top_left = (center_x - box_size // 2, center_y - box_size // 2)
            box_bottom_right = (center_x + box_size // 2, center_y + box_size // 2)
            box_color = (0, 0, 255)  # Varsayılan olarak kırmızı kutu (bulunamadı)

            tag_inside_box = False  # Tag bu kutunun içinde mi?

            if results:  # Eğer tag bulunduysa
                for r in results:
                    # Tag köşe noktalarını al ve tam sayıya çevir
                    (ptA, ptB, ptC, ptD) = r.corners
                    ptA, ptB, ptC, ptD = map(lambda p: tuple(map(int, p)), (ptA, ptB, ptC, ptD))

                    # Tag etrafına yeşil çizgilerle çerçeve çiz
                    cv2.polylines(frame, [np.array([ptA, ptB, ptC, ptD])], isClosed=True, color=(0, 255, 0), thickness=2)

                    # Tag merkezine kırmızı nokta çiz
                    (cX, cY) = tuple(map(int, r.center))
                    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

                    # Tag ID’sini yazdır
                    cv2.putText(frame, f"ID: {r.tag_id}", (ptA[0], ptA[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Tag ile merkez arasına mavi çizgi çiz
                    distance = math.hypot(center_x - cX, center_y - cY)
                    cv2.line(frame, (center_x, center_y), (cX, cY), (255, 0, 0), 2)

                    # Tag’in kapladığı alanı yüzde olarak hesapla
                    tag_area = cv2.contourArea(np.array([ptA, ptB, ptC, ptD]))
                    frame_area = frame_width * frame_height
                    coverage_percentage = (tag_area / frame_area) * 100.0

                    # Eğer tag kutunun içindeyse işaretle
                    if (box_top_left[0] <= cX <= box_bottom_right[0] and
                        box_top_left[1] <= cY <= box_bottom_right[1]):
                        tag_inside_box = True

                    # Yönelme açısını hesapla (ptA -> ptB doğrultusunda)
                    vector_tag = np.array([ptB[0] - ptA[0], ptB[1] - ptA[1]])
                    vector_reference = np.array([1, 0])  # Referans vektör (yatay)

                    angle_rad = np.arctan2(vector_tag[1], vector_tag[0]) - np.arctan2(vector_reference[1], vector_reference[0])
                    angle_rad = np.arctan2(np.sin(angle_rad), np.cos(angle_rad))  # -π ile π arasında normalize et

                    # Twist mesajı hazırla (sapmalar ve durum bilgisi)
                    self.ar_tag_twist.linear.x = float(cX - center_x) / 100.0          # X ekseninde merkezden sapma
                    self.ar_tag_twist.linear.y = float(cY - center_y) / 100.0          # Y ekseninde merkezden sapma
                    self.ar_tag_twist.linear.z = float(coverage_percentage)            # Görüntüdeki kaplama yüzdesi
                    self.ar_tag_twist.angular.x = 1.0                                   # Tag bulunduğunu belirt
                    self.ar_tag_twist.angular.y = float(r.tag_id)                       # Tag ID bilgisi

                    self.publisher.publish(self.ar_tag_twist)  # Mesajı yayınla

            else:
                # Eğer tag bulunamazsa sıfırla
                self.ar_tag_twist.linear.x = 0.0
                self.ar_tag_twist.linear.y = 0.0
                self.ar_tag_twist.linear.z = 0.0
                self.ar_tag_twist.angular.z = 0.0
                self.ar_tag_twist.angular.x = 0.0
                self.publisher.publish(self.ar_tag_twist)

            # Kutunun rengini duruma göre ayarla
            box_color = (0, 255, 0) if tag_inside_box else (0, 0, 255)
            cv2.rectangle(frame, box_top_left, box_bottom_right, box_color, 3)

            # Görüntüyü göster
            cv2.imshow("Gazebo Cam", frame)
            cv2.waitKey(1)

        except Exception as e:
            # Hata varsa logla
            self.get_logger().error(f"Image processing error: {e}")

# ROS düğümünü başlat
def main(args=None):
    rclpy.init(args=args)
    node = AuroDetector()
    try:
        rclpy.spin(node)  # Düğüm sonsuza kadar çalışsın
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  # OpenCV penceresini kapat
        rclpy.shutdown()         # ROS'u kapat

# Ana fonksiyonu çalıştır
if __name__ == "__main__":
    main()
