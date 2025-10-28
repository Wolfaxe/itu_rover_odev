# ITU Rover Klavye Kontrolü ve Odometri Ödevi

Bu proje, ROS Noetic ortamında Gazebo'da simüle edilen bir TurtleBot3 Burger robotunu klavye (`w,a,s,d`) ile kontrol etmeyi amaçlar. Ayrıca, robotun IMU ve tekerlek odometri verilerini `robot_localization` paketi kullanarak birleştirir (EKF ile) ve sonucu RVIz'de görselleştirir. Bu proje, `iturover_keyboard_control_assignment` ödevinin isterlerini ve "Extra" kısmını karşılamak üzere geliştirilmiştir.

## Amaç


## Genel Bakış

* **Kontrol:** Robot, `w` (ileri), `s` (geri), `a` (sola dön), `d` (sağa dön) tuşları ile kontrol edilir. `ESC` tuşu kontrol düğümünü durdurur.
* **Simülasyon:** Gazebo ortamında, ödev için sağlanan özel bir parkur (`obs_corridor_world.world`) kullanılır.
* **Odometri:** Gazebo'nun `/odom` ve `/imu` verileri, `robot_localization` paketindeki Genişletilmiş Kalman Filtresi (EKF) ile birleştirilerek daha hassas bir `odom` -> `base_footprint` dönüşümü (`tf`) ve filtrelenmiş odometri (`/odometry/filtered`) yayınlanır.
* **Görselleştirme:** RVIz kullanılarak robot modeli, lazer taraması (`/scan`) ve EKF tarafından hesaplanan odometri (`/odom`) aynı anda görüntülenir.
* **Süre Limiti:** Klavye kontrol düğümü, ödev isterlerine uygun olarak başlatıldıktan 60 saniye sonra otomatik olarak duracak şekilde ayarlanmıştır (Kod içinden bu süre değiştirilebilir).
* **Loglama:** Robotun hareketlerindeki değişimler (`rospy.loginfo` ile) terminal ekranına yazdırılır.

## Bağımlılıklar

* **İşletim Sistemi:** Ubuntu 20.04 LTS (Focal Fossa) - (WSL 2 üzerinde test edilmiştir)
* **ROS Sürümü:** ROS Noetic Ninjemys
* **Gerekli ROS Paketleri:**
    * `ros-noetic-desktop-full`: Temel ROS kurulumu, Gazebo, RVIz vb. içerir.
    * `ros-noetic-turtlebot3-simulations`: TurtleBot3 simülasyon modellerini ve dünyalarını içerir.
    * `ros-noetic-robot-localization`: EKF düğümünü içerir.
    * `python3-catkin-tools`: `catkin build` komutu için gereklidir.
    * `git`: Depoyu klonlamak için gereklidir.
* **Python Kütüphaneleri:**
    * `pynput`: Klavye girdilerini okumak için gereklidir.

## Kurulum Adımları

1.  **Gerekli Paketleri Yükle:**
    ```bash
    sudo apt update
    sudo apt install ros-noetic-desktop-full python3-catkin-tools git python3-pynput ros-noetic-turtlebot3-simulations ros-noetic-robot-localization -y
    ```
    *(Not: `ros-noetic-robot-localization` paketi `apt` ile kurulduğunda sorun yaşanırsa, aşağıdaki "Kaynaktan Derleme" adımı gerekebilir.)*

2.  **Catkin Çalışma Alanı Oluştur (Eğer yoksa):**
    ```bash
    mkdir -p ~/rover_ws/src
    cd ~/rover_ws/
    catkin init
    catkin build 
    ```

3.  **Bu Depoyu Klonla:**
    ```bash
    cd ~/rover_ws/src/
    git clone https://github.com/Wolfaxe/itu_rover_odev . 
    # Not: URL'den sonraki '.' işareti, deponun içeriğini doğrudan src klasörüne klonlar.
    # Eğer depoyu kendi adıyla (örn: itu_rover_odev) klonlamak istersen '.' yerine o ismi yaz.
    ```

4.  **(İsteğe Bağlı - `robot_localization` Sorun Verirse Kaynaktan Derle):**
    Eğer `apt` ile kurulan `robot_localization` paketi `ekf_localization_node`'u bulamazsa, aşağıdaki adımlarla kaynaktan derleyin:
    ```bash
    cd ~/rover_ws/src/
    git clone [https://github.com/cra-ros-pkg/robot_localization.git](https://github.com/cra-ros-pkg/robot_localization.git) -b noetic-devel
    cd ~/rover_ws/
    catkin build
    ```

5.  **Çalışma Alanını Derle:**
    ```bash
    cd ~/rover_ws/
    catkin build
    ```

6.  **Ortamı Ayarla (Her yeni terminalde gerekli!):**
    ```bash
    source ~/rover_ws/devel/setup.bash
    ```
    *İpucu: Bu komutu kalıcı hale getirmek için `echo "source ~/rover_ws/devel/setup.bash" >> ~/.bashrc` komutunu çalıştırın.*

7.  **TurtleBot3 Modelini Ayarla (Kalıcı):**
    ```bash
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    source ~/.bashrc
    ```

## Kullanım

1.  **Başlatma:** Tek bir terminal açın ve aşağıdaki komutu çalıştırın:
    ```bash
    source ~/rover_ws/devel/setup.bash # Eğer .bashrc'ye eklemediyseniz
    roslaunch reactive_robot reactive_robot.launch 
    # Eğer dosya adını _ ile bitirdiyseniz: roslaunch reactive_robot reactive_robot.launch_
    ```
2.  **Ne Olacak?:** Bu komut şunları otomatik olarak başlatacaktır:
    * Gazebo simülasyonu (özel parkur ve TurtleBot3 Burger ile).
    * RVIz görselleştirmesi (kaydedilmiş ayarları yükleyerek: robot modeli, lazer taraması, odometri okları).
    * Klavye kontrol düğümü (`keyboard_node.py`).
    * IMU odometri düğümü (`imu_odometry_node.py`).
    * EKF düğümü (`ekf_localization_node`).
3.  **Kontrol:** Gazebo penceresi aktifken `w, a, s, d` tuşlarını kullanarak robotu hareket ettirin. Terminalde hareket loglarını görebilirsiniz. RVIz penceresinde robotun hareketini, lazer verisini ve odometri oklarını izleyebilirsiniz. `ESC` tuşu klavye kontrol düğümünü durdurur (60sn limiti aktifse otomatik durur).
4.  **Sıfırlama:**
    * Sadece Gazebo'daki robot pozisyonunu sıfırlamak için: Gazebo penceresine tıklayın ve `Ctrl + R` yapın.
    * Tüm sistemi (Gazebo, RVIz, kodlar, 60sn sayacı) sıfırlamak için: `roslaunch` komutunu çalıştırdığınız terminalde `Ctrl + C` yapın, sonra yukarı ok tuşuyla komutu geri getirip tekrar çalıştırın.

## Notlar ve Sorun Giderme (WSL 2 Kullanıcıları İçin)

* **Gazebo/RVIz Grafik Sorunları:** Eğer Gazebo veya RVIz açılırken çökerse (`Segmentation fault`), yavaş çalışırsa ("kasma") veya görsel hatalar (robot modelinin görünmemesi, garip renkler/şekiller) oluşursa, sorun büyük ihtimalle WSL 2'nin Windows ekran kartı sürücüleriyle olan uyumsuzluğudur.
    * **Çözüm 1 (Geçici):** Programı başlatmadan önce terminale `export LIBGL_ALWAYS_SOFTWARE=1` komutunu girin. Bu, çizimi CPU ile yapmaya zorlar (yavaşlatır ama çalıştırır). RVIz için launch dosyasına eklenmiştir, gerekirse yorumu kaldırın veya manuel başlatırken kullanın.
    * **Çözüm 2 (Kalıcı):** Windows'ta PowerShell'i yönetici olarak açıp `wsl --update` komutunu çalıştırın ve ekran kartı sürücülerinizi (NVIDIA/AMD/Intel) doğrudan üreticinin web sitesinden indirip **en güncel sürüme** yükseltin. Ardından bilgisayarı yeniden başlatın. Bu genellikle `LIBGL_ALWAYS_SOFTWARE=1` komutuna gerek kalmadan sorunu                               çözer.
