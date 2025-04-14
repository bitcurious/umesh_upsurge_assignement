import rospy
from std_msgs.msg import Header
from communication.msg import actuatorcmd  # Replace with your actual message package

def move_spot_5m():
    rospy.init_node('spot_movement_node', anonymous=True)
    pub = rospy.Publisher('/simulation/actuators_cmds', actuatorcmd, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    # Define the gait parameters
    # Example: Define the joint positions for each leg during the gait cycle
    # This is a simplified example; you may need to fine-tune these values
    gait_cycle = [
        {'fl.hx': 0.0, 'fl.hy': 0.5, 'fl.kn': -1.0, 'fr.hx': 0.0, 'fr.hy': -0.5, 'fr.kn': -1.0, 'hl.hx': 0.0, 'hl.hy': 0.5, 'hl.kn': -1.0, 'hr.hx': 0.0, 'hr.hy': -0.5, 'hr.kn': -1.0},
        # Add more phases of the gait cycle here
    ]

    # Define the number of steps to move 5 meters
    # This depends on the step length and the gait cycle duration
    num_steps = 100  # Example value, adjust based on your robot's step length

    for step in range(num_steps):
        for phase in gait_cycle:
            msg = actuatorcmd()
            msg.header = Header(stamp=rospy.Time.now())
            msg.actuators_name = list(phase.keys())
            msg.pos = list(phase.values())
            msg.kp = [100.0] * len(phase)  # Example stiffness
            msg.kd = [10.0] * len(phase)   # Example damping
            msg.vel = [0.0] * len(phase)   # Example velocity
            msg.torque = [0.0] * len(phase)  # Example torque

            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        move_spot_5m()
    except rospy.ROSInterruptException:
        pass