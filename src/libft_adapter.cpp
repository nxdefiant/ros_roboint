#include "roboint.h"
#include "ros/ros.h"
#include "roboint/Output.h"
#include "roboint/Motor.h"
#include "roboint/Inputs.h"


FT_TRANSFER_AREA *transfer_area = NULL;


void cb_set_output(const ::roboint::OutputConstPtr& msg) {
	if (msg->speed == 0) {
		transfer_area->M_Main &= ~(1<<(msg->num-1));
	} else {
		transfer_area->M_Main |= (1<<(msg->num-1));
	}
	transfer_area->MPWM_Main[msg->num-1] = msg->speed;
}


void cb_set_motor(const ::roboint::MotorConstPtr& msg) {
	unsigned char iDirection = 0;

	if (msg->speed > 0) iDirection = 0x1;
	else if (msg->speed < 0) iDirection = 0x2;

	transfer_area->M_Main &= ~(3<<(msg->num-1)*2);
	transfer_area->M_Main |= iDirection<<(msg->num-1)*2;
	transfer_area->MPWM_Main[(msg->num-1)*2] = msg->speed;
	transfer_area->MPWM_Main[(msg->num-1)*2 + 1] = msg->speed;
}


int main(int argc, char **argv)
{
	FT_HANDLE hFt = NULL;

	InitFtLib();

	// Get List of USB devices
	InitFtUsbDeviceList();

	// Get handle on a device
	if ((hFt = GetFtUsbDeviceHandle(0)) == NULL) {
		fprintf(stderr, "No ft Device found\n");
		return 1;
	}

	// Open connection
	OpenFtUsbDevice(hFt);
	SetFtDistanceSensorMode(hFt, IF_DS_INPUT_DISTANCE, IF_DS_INPUT_TOL_STD, IF_DS_INPUT_TOL_STD, 100, 100, IF_DS_INPUT_REP_STD, IF_DS_INPUT_REP_STD);

	StartFtTransferArea(hFt, NULL);
	transfer_area = GetFtTransferAreaAddress(hFt);

	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "libft_adapter");
	
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	ros::Subscriber sub_set_output = n.subscribe("ft/set_output", 8, cb_set_output);
	ros::Subscriber sub_set_motor = n.subscribe("ft/set_motor", 4, cb_set_motor);

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher chatter_pub = n.advertise<roboint::Inputs>("ft/get_inputs", 1000);
	ros::Rate loop_rate(5);

	while(ros::ok()) {
		roboint::Inputs msg;

		msg.input[0] = 0; // unused, Hardware starts at 1
		for (int i=1; i<=8; i++) {
			msg.input[i] = (transfer_area->E_Main & (1<<(i-1))) >> (i-1);
		}
		msg.d1 = transfer_area->D1;

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		chatter_pub.publish(msg);

		/**
		 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
		 * callbacks will be called from within this thread (the main one).  ros::spin()
		 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
		 */
		ros::spinOnce();

		loop_rate.sleep();
	}

	StopFtTransferArea(hFt);
	
	// Close connection
	CloseFtDevice(hFt);

	CloseFtLib();

	return 0;
}

