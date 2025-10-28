#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import time

# Global değişkenler
pub = None
twist = Twist()
start_time = 0
time_limit = 60  # 60 saniye limiti

# Hız ayarları (bunlarla oynayabilirsin)
linear_speed = 0.5
angular_speed = 1.0

def on_press(key):
    global twist

    try:
        if key.char == 'w':
            twist.linear.x = linear_speed
            rospy.loginfo("Started moving forward")
        elif key.char == 's':
            twist.linear.x = -linear_speed
            rospy.loginfo("Started moving backward")
        elif key.char == 'a':
            twist.angular.z = angular_speed
            rospy.loginfo("Started rotating left")
        elif key.char == 'd':
            twist.angular.z = -angular_speed
            rospy.loginfo("Started rotating right")
    except AttributeError:
        pass # Özel tuşlara (Shift vb.) basılınca hata vermemesi için

def on_release(key):
    global twist

    try:
        # Tuşu bıraktığında durması için
        if key.char == 'w' or key.char == 's':
            twist.linear.x = 0.0
            rospy.loginfo("Stopped linear movement")
        elif key.char == 'a' or key.char == 'd':
            twist.angular.z = 0.0
            rospy.loginfo("Stopped angular movement")
    except AttributeError:
        pass

    # ESC tuşu ile çıkış
    if key == keyboard.Key.esc:
        rospy.loginfo("ESC pressed, stopping node...")
        return False

def main():
    global pub, start_time

    rospy.init_node('keyboard_controller', anonymous=True)

    # Robotun hız komutlarını dinlediği topic: /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    start_time = time.time()
    rospy.loginfo("Keyboard control node started. Press 'w,a,s,d' to move. Press ESC to quit.")
    rospy.loginfo(f"Node will automatically stop after {time_limit} seconds.")

    # Klavye dinleyicisini başlat
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed = current_time - start_time

        # 60 saniye kontrolü
        if elapsed > time_limit:
            rospy.loginfo("Time limit reached. Stopping...")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
            break # Döngüden çık

        # Güncel hız komutunu yayınla
        pub.publish(twist)

        if not listener.running:
            break

        rate.sleep()

    listener.stop()
    rospy.loginfo("Keyboard control node stopped.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
