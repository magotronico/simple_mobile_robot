#!/usr/bin/env python

import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import math

chasis = Marker()
caster = Marker()
wheel_r = Marker()
wheel_l = Marker()
# blue is Z, green is Y, red is X

def init_chasis():
    chasis.header.frame_id = "chassis"
    chasis.header.stamp = rospy.Time.now()

    # Define Namespace
    chasis.id = 0 
    chasis.type = 1 #0 arrow, 1 cube, 2 sphere, 3 cylinder, 4 line strip, 5 line list, 6 cube list, 7 sphere list, 8 points, 9 text, 10 mesh, 11 triangle list
    chasis.action = 0 #0 add/modify, 2 delete, 3 deleteall

    # Define Pos
    chasis.pose.position.x = 0.0
    chasis.pose.position.y = 0.0
    chasis.pose.position.z = 0.0

    # Define Orientation
    chasis.pose.orientation.x = 0.0
    chasis.pose.orientation.y = 0.0
    chasis.pose.orientation.z = 0.0
    chasis.pose.orientation.w = 1.0

    # Define Scale
    chasis.scale.x = 0.5
    chasis.scale.y = 0.5
    chasis.scale.z = 0.2

    # Define Color
    chasis.color.r = 0.0
    chasis.color.g = 1.0
    chasis.color.b = 0.0
    chasis.color.a = 1.0

    # Define Lifetime
    chasis.lifetime = rospy.Duration(0)

def init_caster():
    caster.header.frame_id = "caster"
    caster.header.stamp = rospy.Time.now()

    # Define Namespace
    caster.id = 0 
    caster.type = 2 #0 arrow, 1 cube, 2 sphere, 3 cylinder, 4 line strip, 5 line list, 6 cube list, 7 sphere list, 8 points, 9 text, 10 mesh, 11 triangle list
    caster.action = 0 #0 add/modify, 2 delete, 3 deleteall

    # Define Pos
    caster.pose.position.x = 0.0
    caster.pose.position.y = 0.0
    caster.pose.position.z = 0.0

    # Define Orientation
    caster.pose.orientation.x = 0.0
    caster.pose.orientation.y = 0.0
    caster.pose.orientation.z = 0.0
    caster.pose.orientation.w = 1.0

    # Define Scale
    caster.scale.x = 0.12
    caster.scale.y = 0.12
    caster.scale.z = 0.12

    # Define Color
    caster.color.r = 1.0
    caster.color.g = 0.0
    caster.color.b = 0.0
    caster.color.a = 1.0

    # Define Lifetime
    caster.lifetime = rospy.Duration(0)

def init_wheel_r():
    wheel_r.header.frame_id = "wheel_r"
    wheel_r.header.stamp = rospy.Time.now()

    # Define Namespace
    wheel_r.id = 0 
    wheel_r.type = 3 #0 arrow, 1 cube, 2 sphere, 3 cylinder, 4 line strip, 5 line list, 6 cube list, 7 sphere list, 8 points, 9 text, 10 mesh, 11 triangle list
    wheel_r.action = 0 #0 add/modify, 2 delete, 3 deleteall

    # Define Pos
    wheel_r.pose.position.x = 0.0
    wheel_r.pose.position.y = 0.0
    wheel_r.pose.position.z = 0.0

    # Define Orientation
    wheel_r.pose.orientation.x = 0.0
    wheel_r.pose.orientation.y = 0.0
    wheel_r.pose.orientation.z = 0.0
    wheel_r.pose.orientation.w = 1.0

    # Define Scale
    wheel_r.scale.x = 0.2
    wheel_r.scale.y = 0.2
    wheel_r.scale.z = 0.1

    # Define Color
    wheel_r.color.r = 0.0
    wheel_r.color.g = 0.0
    wheel_r.color.b = 1.0
    wheel_r.color.a = 1.0

    # Define Lifetime
    wheel_r.lifetime = rospy.Duration(0)

def init_wheel_l():
    wheel_l.header.frame_id = "wheel_l"
    wheel_l.header.stamp = rospy.Time.now()

    # Define Namespace
    wheel_l.id = 0 
    wheel_l.type = 3 #0 arrow, 1 cube, 2 sphere, 3 cylinder, 4 line strip, 5 line list, 6 cube list, 7 sphere list, 8 points, 9 text, 10 mesh, 11 triangle list
    wheel_l.action = 0 #0 add/modify, 2 delete, 3 deleteall

    # Define Pos
    wheel_l.pose.position.x = 0.0
    wheel_l.pose.position.y = 0.0
    wheel_l.pose.position.z = 0.0

    # Define Orientation
    wheel_l.pose.orientation.x = 0.0
    wheel_l.pose.orientation.y = 0.0
    wheel_l.pose.orientation.z = 0.0
    wheel_l.pose.orientation.w = 1.0

    # Define Scale
    wheel_l.scale.x = 0.2
    wheel_l.scale.y = 0.2
    wheel_l.scale.z = 0.1

    # Define Color
    wheel_l.color.r = 0.0
    wheel_l.color.g = 0.0
    wheel_l.color.b = 1.0
    wheel_l.color.a = 1.0

    # Define Lifetime
    wheel_l.lifetime = rospy.Duration(0)

def publish_static_transforms():
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Base Link to Chassis (Static)
    base_to_chassis = TransformStamped()
    base_to_chassis.header.stamp = rospy.Time.now()
    base_to_chassis.header.frame_id = 'base_link'
    base_to_chassis.child_frame_id = 'chassis'
    base_to_chassis.transform.translation.x = 0.0
    base_to_chassis.transform.translation.y = 0.0
    base_to_chassis.transform.translation.z = 0.1
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi)
    base_to_chassis.transform.rotation.x = q[0]
    base_to_chassis.transform.rotation.y = q[1]
    base_to_chassis.transform.rotation.z = q[2]
    base_to_chassis.transform.rotation.w = q[3]

    # Chassis to Caster Wheel (Static)
    chassis_to_caster_wheel = TransformStamped()
    chassis_to_caster_wheel.header.stamp = rospy.Time.now()
    chassis_to_caster_wheel.header.frame_id = 'chassis'
    chassis_to_caster_wheel.child_frame_id = 'caster'
    chassis_to_caster_wheel.transform.translation.x = -0.15
    chassis_to_caster_wheel.transform.translation.y = 0.0
    chassis_to_caster_wheel.transform.translation.z = -0.08
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    chassis_to_caster_wheel.transform.rotation.x = q[0]
    chassis_to_caster_wheel.transform.rotation.y = q[1]
    chassis_to_caster_wheel.transform.rotation.z = q[2]
    chassis_to_caster_wheel.transform.rotation.w = q[3]

    # Send static transforms
    static_transforms = [base_to_chassis, chassis_to_caster_wheel]
    static_broadcaster.sendTransform(static_transforms)

def update_dynamic_transforms(event):
    dynamic_broadcaster = tf2_ros.TransformBroadcaster()

    # Get current time to use as a basis for circular motion
    t = rospy.Time.now().to_sec()

    # Circular motion parameters
    radius = 1.0  # Define the radius of the circle
    angular_speed = 0.3 # Define the angular speed (radians per second)

    # Calculate circular motion for the chasis
    angle = angular_speed * t
    x = radius * math.cos(angle)  # X coordinate
    y = radius * math.sin(angle)  # Y coordinate
    orientation_angle = angle + math.pi/2  # Orientation angle to keep the base link tangential

    # Calculate circular motion for the wheels based on the chases velocity and radius motion
    wheel_spin_angle = (t * (angular_speed * radius / 0.2)) % (2 * math.pi)

    # Update base link position and orientation based on circular motion
    world_to_base_link = TransformStamped()
    world_to_base_link.header.stamp = rospy.Time.now()
    world_to_base_link.header.frame_id = 'world'
    world_to_base_link.child_frame_id = 'base_link'
    world_to_base_link.transform.translation.x = x
    world_to_base_link.transform.translation.y = y
    world_to_base_link.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, orientation_angle)
    world_to_base_link.transform.rotation.x = q[0]
    world_to_base_link.transform.rotation.y = q[1]
    world_to_base_link.transform.rotation.z = q[2]
    world_to_base_link.transform.rotation.w = q[3]

    # Chassis to Right Wheel (Dynamic)
    chassis_to_right_wheel = TransformStamped()
    chassis_to_right_wheel.header.stamp = rospy.Time.now()
    chassis_to_right_wheel.header.frame_id = 'chassis'
    chassis_to_right_wheel.child_frame_id = 'wheel_r'
    # These values would be dynamically calculated
    chassis_to_right_wheel.transform.translation.x = 0.1
    chassis_to_right_wheel.transform.translation.y = 0.3
    chassis_to_right_wheel.transform.translation.z = -0.05
    q = tf_conversions.transformations.quaternion_from_euler(math.pi/2, -wheel_spin_angle, 0)
    chassis_to_right_wheel.transform.rotation.x = q[0]
    chassis_to_right_wheel.transform.rotation.y = q[1]
    chassis_to_right_wheel.transform.rotation.z = q[2]
    chassis_to_right_wheel.transform.rotation.w = q[3]

    # Chassis to Left Wheel (Dynamic)
    chassis_to_left_wheel = TransformStamped()
    chassis_to_left_wheel.header.stamp = rospy.Time.now()
    chassis_to_left_wheel.header.frame_id = 'chassis'
    chassis_to_left_wheel.child_frame_id = 'wheel_l'
    # These values would be dynamically calculated
    chassis_to_left_wheel.transform.translation.x = 0.1
    chassis_to_left_wheel.transform.translation.y = -0.3
    chassis_to_left_wheel.transform.translation.z = -0.05
    q = tf_conversions.transformations.quaternion_from_euler(math.pi/2, -wheel_spin_angle, 0)
    chassis_to_left_wheel.transform.rotation.x = q[0]
    chassis_to_left_wheel.transform.rotation.y = q[1]
    chassis_to_left_wheel.transform.rotation.z = q[2]
    chassis_to_left_wheel.transform.rotation.w = q[3]

    # Send dynamic transforms
    dynamic_transforms = [world_to_base_link, chassis_to_right_wheel, chassis_to_left_wheel]
    dynamic_broadcaster.sendTransform(dynamic_transforms)

def start_dynamic_broadcaster():
    rospy.Timer(rospy.Duration(0.01), update_dynamic_transforms)

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Shutting down node")

def main():
    rospy.init_node('robot_state_publisher')
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)

    pub_chasis = rospy.Publisher('/chasis', Marker, queue_size=10)
    rospy.loginfo("Publishing Chasis Marker")
    init_chasis()

    pub_caster = rospy.Publisher('/caster', Marker, queue_size=10)
    rospy.loginfo("Publishing Caster Marker")
    init_caster()

    pub_wheel_r = rospy.Publisher('/wheel_r', Marker, queue_size=10)
    rospy.loginfo("Publishing Right Wheel Marker")
    init_wheel_r()

    pub_wheel_l = rospy.Publisher('/wheel_l', Marker, queue_size=10)
    rospy.loginfo("Publishing Left Wheel Marker")
    init_wheel_l()

    publish_static_transforms()
    start_dynamic_broadcaster()
    
    try:
        while not rospy.is_shutdown():
            chasis.header.stamp = rospy.Time.now()
            caster.header.stamp = rospy.Time.now()
            wheel_r.header.stamp = rospy.Time.now()
            wheel_l.header.stamp = rospy.Time.now()

            pub_chasis.publish(chasis)
            pub_caster.publish(caster)
            pub_wheel_r.publish(wheel_r)
            pub_wheel_l.publish(wheel_l)
            
            loop_rate.sleep()
    except:
        pass

if __name__ == '__main__':
    main()
