from launch import LaunchDescription
from launch_ros.actions import Node

# /cmd_vel konusunu ROS <-> Gazebo arasında köprüleyen node tanımı.
# ROS tarafında Twist mesajı ile kontrol komutları gönderilir,
# Gazebo tarafında bu komutlar gz.msgs.Twist olarak iletilir.
bridge_control = Node(
    package="ros_gz_bridge",  # Gazebo ile ROS 2 arasında köprü kuran paket (ros_ign_bridge eskiden kullanılıyordu)
    executable="parameter_bridge",  # Mesaj tipleri arasında çeviri yapan yürütülebilir dosya
    arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],  # Konu adı ve mesaj tiplerinin eşlemesi (Not: köşeli parantez hatalı olabilir, aşağıda açıklayacağım)
    output="log"  # Konsola log çıktısı ver
)

# Klavyeden gelen tuş basımlarını ROS <-> Gazebo arasında köprüleyen node tanımı.
# Bu örnekte klavyeden gelen girdiler Gazebo tarafından simüle edilir ve ROS tarafında std_msgs/Int32 olarak alınır.
bridge_keyboard = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32"],  # Konu adı ve mesaj eşlemesi
    output="log"
)

# Kamera görüntüsünü ROS <-> Gazebo arasında köprüleyen node tanımı.
# Kamera verisi Gazebo'dan gz.msgs.Image formatında gelir, ROS tarafında sensor_msgs/Image olarak alınır.
bridge_camera = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/camera@sensor_msgs/msg/Image[gz.msgs.Image"],  # Görüntü mesajı eşlemesi
    output="log"
)

# Yukarıda tanımlanan üç köprü node'unu başlatacak olan launch dosyası
def generate_launch_description():
    return LaunchDescription([
        bridge_control,
        bridge_keyboard,
        bridge_camera
    ])
