/**
 * @file offboard_control_node.hpp
 * @brief Offboard kontrol mantığını yöneten ROS2 düğümü.
 */

 #ifndef OFFBOARD_CONTROL_NODE_HPP
 #define OFFBOARD_CONTROL_NODE_HPP
 
 #include "commander.hpp"
 
 /**
  * @class OffboardControlNode
  * @brief PX4 ile offboard kontrol sağlayan ROS2 düğüm sınıfı.
  */
 class OffboardControlNode : public rclcpp::Node
 {
 public:
     /**
      * @brief OffboardControlNode sınıfı kurucusu.
      * ROS2 node'u başlatır ve zamanlayıcıları kurar.
      */
     OffboardControlNode();
 
     /**
      * @brief Aracı arm (uçuşa hazır) hale getirir.
      */
     void arm();
 
     /**
      * @brief Aracın arm durumunu kapatır (disarm).
      */
     void disarm();
 
     /**
      * @brief Acil durumda aracı durdurur (kill switch).
      */
     void killSwitch();
 
     /**
      * @brief Zamanlayıcı ile tetiklenen komut güncelleme döngüsü.
      */
     void commandCallback();
 
 private:
     /**
      * @brief Node'u başlatır: publisher/subscriber'ları oluşturur, parametreleri okur.
      */
     void init();
 
     /**
      * @brief Gerekli publisher nesnelerini oluşturur.
      */
     void initPublishers();
 
     /**
      * @brief Gerekli subscriber nesnelerini oluşturur.
      */
     void initSubscribers();
 
     /**
      * @brief PID kontrol parametrelerini ROS parametreleri olarak okur.
      */
     void declerationParameters();
 
     /**
      * @brief Araç durumu güncellemelerini işler.
      * @param msg Araç durumu mesajı.
      */
     void vehicleStatusCallback(const vehicleStatusMsgPtr msg);
 
     /**
      * @brief Aracın odometri bilgisini işler.
      * @param msg Araç odometri mesajı.
      */
     void vehicleOdometryCallback(const vehicleOdometryMsgPtr msg);
 
     /**
      * @brief Fail-safe durumlarını işler (şu an kullanılmıyor).
      * @param msg Fail-safe bayrak mesajı.
      */
     void failsafeFlagsCallback(const failsafeFlagsMsgPtr msg);
 
     /**
      * @brief Klavye girdilerini işler.
      * @param msg Klavye tuş mesajı.
      */
     void keyPressCallback(const int32MsgPtr msg);
 
     /**
      * @brief ArUco işaretleyici pozisyon bilgisini işler.
      * @param msg Twist mesajı (pozisyon ve yön hatası içerir).
      */
     void arucoPoseCallback(const twistMsgPtr msg);
 
     /**
      * @brief Offboard kontrol modu mesajı yayınlar.
      */
     void offboardControlModePublish();
 
     /**
      * @brief Hedef (trajectory) setpoint yayınlar.
      */
     void trajectorySetpointPublish();
 
     /**
      * @brief PX4'e komut mesajı gönderir.
      * @param command Komut tipi.
      * @param param1 Komut parametresi 1.
      * @param param2 Komut parametresi 2.
      */
     void vehicleCommandPublish(uint16_t command, float param1 = 0.0, float param2 = 0.0);
 
     /**
      * @brief Kara aracı hız komutu yayınlar.
      */
     void landVehicleVelocityPublish();
 
 private:
     Timers timers_;                      ///< Zamanlayıcı nesneleri.
     Publishers publishers_;              ///< Publisher koleksiyonu.
     Subscribers subscribers_;            ///< Subscriber koleksiyonu.
     VehicleStatus_s vehicle_status_;     ///< Araç durumu.
     Odometry_s odometry_;                ///< Aracın konumu ve yönelimi.
     ArucoTypes_s aruco_;                 ///< ArUco hedef bilgisi.
     Odometry_s setpoint_;                ///< Hedef konum.
     Odometry_s land_vehcile_setpoint_;   ///< Kara aracı için hedef hızlar.
     SystemStatus_s system_status_;       ///< Genel sistem durumu.
     PID_s pid_;                          ///< PID kontrol parametreleri.
     std::shared_ptr<Commander> _commander; ///< Komut yönetim nesnesi.
 };
 
 #endif // OFFBOARD_CONTROL_NODE_HPP
 