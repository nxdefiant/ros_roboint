#include <stdlib.h>
#include <pthread.h>
#include "roboint.h"
#include "ros/ros.h"
#include "roboint/Output.h"
#include "roboint/Motor.h"
#include "roboint/Inputs.h"


static FT_TRANSFER_AREA *transfer_area = NULL;
static char pwm[8] = {0};
static char pwm_next[8] = {0};
static pthread_mutex_t pwm_mutex = PTHREAD_MUTEX_INITIALIZER;

void cb_set_output(const ::roboint::OutputConstPtr& msg) {
	pthread_mutex_lock(&pwm_mutex);
	pwm_next[msg->num] = abs(msg->speed);
	pthread_mutex_unlock(&pwm_mutex);
}


void cb_set_motor(const ::roboint::MotorConstPtr& msg) {
	pthread_mutex_lock(&pwm_mutex);
	if (msg->speed > 0) {
		pwm_next[msg->num*2] = abs(msg->speed);
		pwm_next[msg->num*2+1] = 0;
	} else {
		pwm_next[msg->num*2] = 0;
		pwm_next[msg->num*2+1] = abs(msg->speed);
	}
	pthread_mutex_unlock(&pwm_mutex);
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
	ros::Publisher pub_inputs = n.advertise<roboint::Inputs>("ft/get_inputs", 10);
	ros::Rate loop_rate(100);

	while(ros::ok()) {
		roboint::Inputs msg;

		pthread_mutex_lock(&pwm_mutex);
		msg.header.stamp = ros::Time::now();
		sem_wait(&hFt->lock);
		for (int i=0; i<=7; i++) {
			msg.input[i] = (transfer_area->E_Main & (1<<i)) >> i;
		}
		for (int i=0; i<=7; i++) {
			msg.output[i] = pwm[i];
			pwm[i] = pwm_next[i];

			if (pwm[i] == 0) {
				transfer_area->M_Main &= ~(1<<(i));
			} else {
				transfer_area->M_Main |= (1<<(i));
			}
			transfer_area->MPWM_Main[i] = pwm[i];
		}
		msg.ax = transfer_area->AX;
		msg.ay = transfer_area->AY;
		msg.a1 = transfer_area->A1;
		msg.a2 = transfer_area->A2;
		msg.av = transfer_area->AV;
		msg.d1 = transfer_area->D1;
		msg.d2 = transfer_area->D2;
		sem_post(&hFt->lock);
		pthread_mutex_unlock(&pwm_mutex);

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		pub_inputs.publish(msg);

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

