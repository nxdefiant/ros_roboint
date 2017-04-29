#include "ros/ros.h"
#include "robo_explorer_hardware.h"
#include "controller_manager/controller_manager.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "robo_explorer_hardware");

	ros::NodeHandle n;

	RoboExplorer robot(n);
	controller_manager::ControllerManager cm(&robot);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Time prev_time = ros::Time::now();
	ros::Rate loop_rate(10);
	while(ros::ok()) {
		const ros::Time time = ros::Time::now();
		const ros::Duration period = time - prev_time;

		robot.read(period);
		cm.update(time, period);
		robot.write();

		prev_time = time;
		loop_rate.sleep();
	}

	return 0;
}
