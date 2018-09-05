import nstbot
import rospy
from std_msgs.msg import Float64MultiArray


rospy.init_node('pushbot')
bot = nstbot.PushBotRos()
bot.connect(nstbot.Serial('/dev/ttyUSB0', baud=4000000))
# bot.connect(nstbot.Socket('192.168.1.95'))
bot.retina(True, bytes_in_timestamp=0)
bot.activate_sensors(period=0.1, accel=True, gyro=True,
                     event_rate=True, motor_ticks=True, linear_acc=True, heading=True
)
action_pub = rospy.Publisher('action', Float64MultiArray, queue_size=1)

r = rospy.Rate(50)
while not rospy.is_shutdown():
    msg = Float64MultiArray(
        data=[-0.1, 0.1]
    )
    action_pub.publish(msg)
    r.sleep()
