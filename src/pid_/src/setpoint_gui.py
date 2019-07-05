import rospy
from std_msgs.msg import Float32
from tkinter import *

left_leg_pub = rospy.Publisher('left_leg_setpoint', Float32, queue_size = 10)
left_thigh_pub = rospy.Publisher('left_thigh_setpoint', Float32, queue_size = 10)
right_leg_pub = rospy.Publisher('right_leg_setpoint', Float32, queue_size = 10)
right_thigh_pub = rospy.Publisher('right_thigh_setpoint', Float32, queue_size = 10)


data = [0, 0, 0, 0]

def Send_Data():
    data[0] = A1.get()
    data[1] = A2.get()
    data[2] = A3.get()
    data[3] = A4.get()
    
    left_leg_pub.publish(data[0])
    left_thigh_pub.publish(data[1])
    right_leg_pub.publish(data[2])
    right_thigh_pub.publish(data[3])

    print(data)

master = Tk()

master.title('Sending Angels using ROS')

Label(master, text='Left  Leg').grid(row=0) 
Label(master, text='Left  Thigh').grid(row=1)
Label(master, text='Right Leg').grid(row=2)
Label(master, text='Right Thigh').grid(row=3)

A1 = Entry(master, justify='center')
A2 = Entry(master, justify='center')
A3 = Entry(master, justify='center')
A4 = Entry(master, justify='center')

A1.grid(row=0, column=1)
A2.grid(row=1, column=1)
A3.grid(row=2, column=1)
A4.grid(row=3, column=1)

sendButton = Button(master, text='Send', width=25, fg='green', command=Send_Data).grid(row=4, column=0)
exitButton = Button(master, text='Exit', width=25, fg='red', command=master.destroy).grid(row=4, column=1)


mainloop() 
