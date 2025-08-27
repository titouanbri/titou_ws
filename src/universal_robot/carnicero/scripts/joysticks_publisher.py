#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped  


class JoystickAggregator:
    def __init__(self):
        rospy.init_node('joystick_aggregator_node', anonymous=True)
        
        # Initialisation des données des joysticks
        self.joy1_axes = [0.0] * 6
        self.joy1_buttons = [0] * 10
        self.joy2_axes = [0.0] * 6
        self.joy2_buttons = [0] * 10
        
        # Publishers
        self.output_pub = rospy.Publisher('/joysticks_publisher', WrenchStamped, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/joy1', Joy, self.joy1_callback)
        rospy.Subscriber('/joy2', Joy, self.joy2_callback)
        
        # Taux de publication
        self.rate = rospy.Rate(500)  # 50 Hz
        
    def joy1_callback(self, data):
        self.joy1_axes = data.axes
        self.joy1_buttons = data.buttons
        
    def joy2_callback(self, data):
        self.joy2_axes = data.axes
        self.joy2_buttons = data.buttons

    def run(self):
        Gt=10
        Gr=1
        while not rospy.is_shutdown():
            # Création du message combiné
            combined_msg = WrenchStamped()
            
            # Mapping des 3 translations (joystick 1)
            translation_x =Gt* self.joy1_axes[0]
            translation_y =Gt* (self.joy1_axes[1]-0.56)*2
            translation_z =0 #G* self.joy1_axes[2]
            
            # Mapping des 3 rotations (joystick 2)
            rotation_x = Gr* self.joy2_axes[0]
            rotation_y = Gr* self.joy2_axes[1]
            rotation_z = 0 #G* self.joy2_axes[2]
            
            # Logique pour les boutons
            # Bouton 1 du joystick 1
            if len(self.joy1_buttons) > 0 and self.joy1_buttons[0] == 1:
                translation_z = Gt
            
            
            # Bouton 2 du joystick 1
            if len(self.joy1_buttons) > 1 and self.joy1_buttons[1] == 1:
                translation_z = -Gt
                
            # Bouton 2 du joystick 2
            if len(self.joy2_buttons) > 1 and self.joy2_buttons[1] == 1:
                rotation_z = Gr
            
            # Bouton 3 du joystick 2
            if len(self.joy2_buttons) > 2 and self.joy2_buttons[2] == 1:
                rotation_z = -Gr
            
            if abs(translation_y)<1:
                translation_y =0

            combined_msg.header.stamp = rospy.Time.now()
            # combined_msg.header.frame_id = msg.header.frame_id
            combined_msg.wrench.force.x=translation_x
            combined_msg.wrench.force.y=translation_y
            combined_msg.wrench.force.z=translation_z
            combined_msg.wrench.torque.x=rotation_x
            combined_msg.wrench.torque.y=rotation_y
            combined_msg.wrench.torque.z=rotation_z

        
            # Publication du message
            self.output_pub.publish(combined_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        aggregator = JoystickAggregator()
        aggregator.run()
    except rospy.ROSInterruptException:
        pass