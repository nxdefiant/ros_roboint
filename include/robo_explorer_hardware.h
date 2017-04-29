#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "roboint/Inputs.h"
#include "roboint/Motor.h"

class RoboExplorer : public hardware_interface::RobotHW
{
	public:
		RoboExplorer(ros::NodeHandle nh) { 
			this->nh = nh;
			pub_motor = nh.advertise<roboint::Motor>("ft/set_motor", 10);
			sub_inputs = nh.subscribe("ft/get_inputs", 10, &RoboExplorer::cbInputs, this);

			// connect and register the joint state interface
			hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
			jnt_state_interface.registerHandle(state_handle_left);
			hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
			jnt_state_interface.registerHandle(state_handle_right);
			registerInterface(&jnt_state_interface);

			// connect and register the joint velocity interface
			hardware_interface::JointHandle joint_handle_left(state_handle_left, &cmd[0]);
			jnt_velocity_interface.registerHandle(joint_handle_left);
			hardware_interface::JointHandle joint_handle_right(state_handle_right, &cmd[1]);
			jnt_velocity_interface.registerHandle(joint_handle_right);
			registerInterface(&jnt_velocity_interface);
		}

		// Converts pos_encoder from wheels to wheel angles
		void read(ros::Duration period) {
			static double pos_last[2];

			pos[0] = pos_encoder[0] * M_PI/8;
			pos[1] = pos_encoder[1] * M_PI/8;
			vel[0] = (pos[0] - pos_last[0])/period.toSec();
			vel[1] = (pos[1] - pos_last[1])/period.toSec();
			
			std::copy(std::begin(pos), std::end(pos), std::begin(pos_last));
		}

		// Writes current velocity command to hardware
		void write() {
			double speed_l = 0;
			double speed_r = 0;
			double wish_speed_left = cmd[0];
			double wish_speed_right = cmd[1];
			roboint::Motor msg;

			if (fabs(wish_speed_left) > 0) {
				speed_l = 64.3*fabs(wish_speed_left) -1.7;
				if (wish_speed_left < 0) speed_l*=-1;
			}
			if (fabs(wish_speed_right) > 0) {
				speed_r = 64.3*fabs(wish_speed_right) -1.7;
				if (wish_speed_right < 0) speed_r*=-1;
			}

			// check limits
			if (speed_l < -7) speed_l = -7;
			else if (speed_l > 7) speed_l = 7;
			if (speed_r < -7) speed_r = -7;
			else if (speed_r > 7) speed_r = 7;

			msg.num = 0;
			msg.speed = round(speed_l);
			pub_motor.publish(msg);

			msg.num = 1;
			msg.speed = round(speed_r);
			pub_motor.publish(msg);
		}

		// Reads current input state and increases pos_encoder on change
		void cbInputs(const roboint::Inputs::ConstPtr& msg) {
			static std::array<uint8_t, 8> input_last;

			if (msg->input[0] != input_last[0]) { // left input changed
				// outputs have direction information
				if (msg->output[0] > 0) pos_encoder[0]++;
				else if (msg->output[1] > 0) pos_encoder[0]--;
			}
			if (msg->input[1] != input_last[1]) { // right input changed
				// outputs have direction information
				if (msg->output[2] > 0) pos_encoder[1]++;
				else if (msg->output[3] > 0) pos_encoder[1]--;
			}

			std::copy(std::begin(msg->input), std::end(msg->input), std::begin(input_last));
		}

	private:
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::VelocityJointInterface jnt_velocity_interface;
		ros::NodeHandle nh;
		ros::Publisher pub_motor;
		ros::Subscriber sub_inputs;
		double cmd[2];
		double pos[2];
		double vel[2];
		double eff[2];
		int64_t pos_encoder[2] = {0, 0};
};
