import rospy
from std_srvs.srv import TriggerRequest, Trigger
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from geometry_msgs.msg import WrenchStamped, Twist

P = 1
I = 1
D = 1
FT_SETPOINT = 4.0
ERROR_ALLOWANCE = 1.0

force_z = None
command_vel = None

def PID():
    global command_vel
    error = FT_SETPOINT - force_z
    if error > ERROR_ALLOWANCE:
        command_vel.linear.z = -0.01
    else :
        command_vel.linear.z = 0.0

def wrench_cb(msg:WrenchStamped):
    global force_z
    force_z = msg.wrench.force.z

rospy.init_node("ft_pick")
rospy.loginfo("Registering Services")
twist_command_publisher = rospy.Publisher("/left/twist_controller/command",Twist,queue_size=10)
zero_ftsensor = rospy.ServiceProxy('/left/ur_hardware_interface/zero_ftsensor', Trigger)
zero_ftsensor.wait_for_service()
switch_controller = rospy.ServiceProxy('/left/controller_manager/switch_controller', SwitchController)
switch_controller.wait_for_service()
rospy.Subscriber("/left/wrench",WrenchStamped,callback=wrench_cb)
rospy.wait_for_message("/left/wrench",WrenchStamped)
rospy.loginfo("All services registered")

switch_controller_msg = SwitchControllerRequest()
switch_controller_msg.start_controllers = ["twist_controller"]
switch_controller_msg.stop_controllers = ["scaled_pos_joint_traj_controller"]
switch_controller_msg.strictness = switch_controller_msg.STRICT
switch_controller_msg.start_asap = True

rospy.loginfo("Switching controller from scaled_pos_joint_traj_controller to twist_controller")
try:
    switch_controller_response = switch_controller(switch_controller_msg)
except Exception as e:
    rospy.logerr(f"error occurred while switching controllers = {e}")
rospy.loginfo("Controller switched")

rate = rospy.Rate(30)
command_vel = Twist()
rospy.loginfo("Moving downwards now")
trigger = TriggerRequest()

rospy.loginfo("Zeroeing ft sensor")
try:
    zero_ftsensor_response = zero_ftsensor(trigger)
except Exception as e:
    rospy.logerr(f"error occurred while switching controllers = {e}")

i=0

while not rospy.is_shutdown():
    PID()
    twist_command_publisher.publish(command_vel)
    if command_vel.linear.z==0:
        i+=1
        if i>10:
            break
    rate.sleep()

switch_controller_msg = SwitchControllerRequest()
switch_controller_msg.start_controllers = ["scaled_pos_joint_traj_controller"]
switch_controller_msg.stop_controllers = ["twist_controller"]
switch_controller_msg.strictness = switch_controller_msg.STRICT
switch_controller_msg.start_asap = True

rospy.loginfo("Arrived, giving back control to scaled_pos_joint_traj_controller")
try:
    switch_controller_response = switch_controller(switch_controller_msg)
except Exception as e:
    rospy.logerr(f"error occurred while switching controllers = {e}")
rospy.loginfo("Controller switched")