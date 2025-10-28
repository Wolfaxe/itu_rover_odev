#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# Global olarak publisher'ı tanımlayalım
odom_pub = None

def imu_callback(imu_msg):
    global odom_pub

    # Yeni bir Odometry mesajı oluştur
    odom_msg = Odometry()

    # IMU mesajından gelen bilgileri Odometry mesajına kopyala

    # 1. Header'ı (zaman damgası vb.) kopyala
    odom_msg.header = imu_msg.header

    # 2. Odometri mesajı için "frame"leri (çerçeveleri) ayarla
    #    'header.frame_id' bizim ana koordinat sistemimiz (örn: 'odom')
    #    'child_frame_id' ise robotun kendi çerçevesi (örn: 'base_link')
    #    (Bu isimler robotun URDF modeline göre değişir, 'base_link' standarttır)
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # 3. IMU'dan gelen Oryantasyon (duruş) bilgisini kopyala
    odom_msg.pose.pose.orientation = imu_msg.orientation

    # 4. IMU'dan gelen Açısal Hız bilgisini kopyala
    odom_msg.twist.twist.angular = imu_msg.angular_velocity

    # 5. Kovaryans matrislerini AYARLA (IMU'dan kopyalamak yerine)
    # Odometry 6x6 (36 eleman), IMU 3x3 (9 eleman) kovaryans kullanır.
    # Sadece IMU'nun bildiği kısımları (oryantasyon ve açısal hız) dolduracağız.

    # Pose Kovaryansı (6x6):
    # Sadece oryantasyon (Roll, Pitch, Yaw) kovaryansını IMU'dan alalım.
    # Pozisyon (X, Y, Z) için varsayılan küçük bir değer atayalım.
    odom_msg.pose.covariance = [1e-3, 0, 0, 0, 0, 0, # X için küçük hata
                                0, 1e-3, 0, 0, 0, 0, # Y için küçük hata
                                0, 0, 1e6, 0, 0, 0, # Z için BÜYÜK hata (bilmiyoruz)
                                0, 0, 0, imu_msg.orientation_covariance[0], imu_msg.orientation_covariance[1], imu_msg.orientation_covariance[2], # Roll
                                0, 0, 0, imu_msg.orientation_covariance[3], imu_msg.orientation_covariance[4], imu_msg.orientation_covariance[5], # Pitch
                                0, 0, 0, imu_msg.orientation_covariance[6], imu_msg.orientation_covariance[7], imu_msg.orientation_covariance[8]] # Yaw

    # Twist Kovaryansı (6x6):
    # Sadece açısal hız (Roll, Pitch, Yaw) kovaryansını IMU'dan alalım.
    # Lineer hız (X, Y, Z) için varsayılan küçük bir değer atayalım.
    odom_msg.twist.covariance = [1e-3, 0, 0, 0, 0, 0, # Vx için küçük hata
                                 0, 1e-3, 0, 0, 0, 0, # Vy için küçük hata
                                 0, 0, 1e6, 0, 0, 0, # Vz için BÜYÜK hata (bilmiyoruz)
                                 0, 0, 0, imu_msg.angular_velocity_covariance[0], imu_msg.angular_velocity_covariance[1], imu_msg.angular_velocity_covariance[2], # Roll hızı
                                 0, 0, 0, imu_msg.angular_velocity_covariance[3], imu_msg.angular_velocity_covariance[4], imu_msg.angular_velocity_covariance[5], # Pitch hızı
                                 0, 0, 0, imu_msg.angular_velocity_covariance[6], imu_msg.angular_velocity_covariance[7], imu_msg.angular_velocity_covariance[8]] # Yaw hızı


    # Not: IMU bize lineer pozisyon (x,y) veya lineer hız vermez.
    # Bu yüzden odom_msg.pose.pose.position ve odom_msg.twist.twist.linear
    # alanlarını varsayılan olarak (0,0,0) bırakıyoruz.
    # "Odometri yap"tan kasıt, IMU verisini Odometri formatına çevirmektir.

    # 6. Hazırladığımız Odometry mesajını yayınla
    if odom_pub is not None:
        odom_pub.publish(odom_msg)

def main():
    global odom_pub

    rospy.init_node('imu_odometry_node', anonymous=True)

    # /imu_odometry adında yeni bir topic'e yayın yapacak bir publisher oluştur
    odom_pub = rospy.Publisher('/imu_odometry', Odometry, queue_size=10)

    # /imu topic'ini dinleyecek bir subscriber oluştur
    # Her yeni /imu mesajı geldiğinde 'imu_callback' fonksiyonunu çalıştır
    rospy.Subscriber('/imu', Imu, imu_callback)

    rospy.loginfo("IMU to Odometry node started. Publishing to /imu_odometry")

    # Node'un kapanana kadar çalışmasını sağla
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
