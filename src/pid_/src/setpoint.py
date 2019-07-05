import rospy
from std_msgs.msg import Float32

# store the angles
left_leg_angle = 0
left_thigh_angle = 0
right_leg_angle = 0
right_thigh_angle = 0

rospy.init_node('setpoint_publisher', anonymous=False)


def update_left_leg_angle(data):
    global left_leg_angle
    left_leg_angle = data.data


def update_left_thigh_angle(data):
    global left_thigh_angle
    left_thigh_angle = data.data


def update_right_leg_angle(data):
    global right_leg_angle
    right_leg_angle = data.data


def update_right_thigh_angle(data):
    global right_thigh_angle
    right_thigh_angle = data.data


rospy.Subscriber("left_leg_angle", Float32, callback=update_left_leg_angle)
rospy.Subscriber("left_thigh_angle", Float32, callback=update_left_thigh_angle)
rospy.Subscriber("right_leg_angle", Float32, callback=update_right_leg_angle)
rospy.Subscriber("right_thigh_angle", Float32, callback=update_right_thigh_angle)

# setup publishers
left_leg_setpoint = rospy.Publisher("left_leg_setpoint", Float32, queue_size=10)
left_thigh_setpoint = rospy.Publisher("left_thigh_setpoint", Float32, queue_size=10)
right_leg_setpoint = rospy.Publisher("right_leg_setpoint", Float32, queue_size=10)
right_thigh_setpoint = rospy.Publisher("right_thigh_setpoint", Float32, queue_size=10)


def publish_data(left_leg_angleT, left_thigh_angleT, right_leg_angleT, right_thigh_angleT):
    left_leg_setpoint.publish(left_leg_angleT)
    left_thigh_setpoint.publish(left_thigh_angleT)
    right_leg_setpoint.publish(right_leg_angleT)
    right_thigh_setpoint.publish(right_thigh_angleT)


def walk():
    publish_data(90, 0, 90, 0)


def sit():
    publish_data(90, 0, 90, 0)


def start():
    x = input("s to sit, m to walk")
    if x == 's':
        sit()
    elif x == 'm':
        walk()
    else:
        print('خخخخخخخخخخخخ')


if __name__ == '__main__':
    start()
