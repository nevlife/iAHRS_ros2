#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <csignal>
#include <atomic>
#include <iostream>

// POSIX headers for serial communication
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_srvs/srv/trigger.hpp"

namespace iahrs_driver
{

// Using modern C++ style for constants
static const speed_t SERIAL_SPEED = B115200;
static const int COMM_RECV_TIMEOUT_MS = 30;

// Replaced struct with a class for better encapsulation, though struct is also fine.
// Using PascalCase for type names.
struct ImuData
{
	double angular_velocity_x = 0.0;
	double angular_velocity_y = 0.0;
	double angular_velocity_z = 0.0;
	
	double linear_acceleration_x = 0.0;
	double linear_acceleration_y = 0.0;
	double linear_acceleration_z = 0.0;
    
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
};

// Renamed class to PascalCase
class IahrsDriver : public rclcpp::Node
{
public:
	IahrsDriver() : Node("iahrs_driver")
	{
		// Initialize parameters
		this->declare_parameter<std::string>("port", "/dev/iAHRS");
		this->declare_parameter<std::string>("parent_frame_id", "base_link");
		this->declare_parameter<bool>("publish_tf", true);
		this->declare_parameter<double>("tf_pos_x", 0.0);
		this->declare_parameter<double>("tf_pos_y", 0.0);
		this->declare_parameter<double>("tf_pos_z", 0.2);
		
		port_ = this->get_parameter("port").as_string();
		parent_frame_id_ = this->get_parameter("parent_frame_id").as_string();
		publish_tf_ = this->get_parameter("publish_tf").as_bool();
		tf_pos_x_ = this->get_parameter("tf_pos_x").as_double();
		tf_pos_y_ = this->get_parameter("tf_pos_y").as_double();
		tf_pos_z_ = this->get_parameter("tf_pos_z").as_double();

		// Initialize publisher and broadcaster
		imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("iahrs_imu/data", 10);
		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		auto euler_angle_reset_callback =
			[this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
				   std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
				(void)request;
				char cmd[20] = "$sc,rst\r\n";
				int cmd_len = strlen(cmd);
				int ret = write(this->serial_fd_, cmd, cmd_len);
				if (ret == cmd_len) {
					response->success = true;
					response->message = "Euler angle reset command sent.";
					RCLCPP_INFO(this->get_logger(), "Euler angle reset command sent.");
				} else {
					response->success = false;
					response->message = "Failed to send Euler angle reset command.";
					RCLCPP_ERROR(this->get_logger(), "Failed to send Euler angle reset command.");
				}
			};

		euler_angle_reset_service_ =
			this->create_service<std_srvs::srv::Trigger>("~/euler_angle_reset", euler_angle_reset_callback);

		// Set fixed covariance values
		set_covariance_values();

		RCLCPP_INFO(this->get_logger(), "IAHRS Driver Node has been initialized.");
	}

	// Public method to open the serial port and start processing
	void run()
	{
		if (!open_serial_port()) {
			RCLCPP_FATAL(this->get_logger(), "Failed to open serial port. Shutting down.");
			rclcpp::shutdown();
			return;
		}

		// Reset euler angles on startup
		reset_device_euler_angles();
		RCLCPP_INFO(this->get_logger(), "Device Euler angles have been reset.");
		
		print_ascii_art();

		// Main loop using a WallRate
		rclcpp::WallRate loop_rate(100); // 100 Hz
		while(rclcpp::ok()) {
			process_imu_data();
			rclcpp::spin_some(this->get_node_base_interface());
			loop_rate.sleep();
		}

		// Clean up on shutdown
		close(serial_fd_);
	}

private:
	// Renamed methods to snake_case
	bool open_serial_port()
	{
		RCLCPP_INFO(this->get_logger(), "Trying to open serial port: %s", port_.c_str()); 

		serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
		if (serial_fd_ < 0) 
		{
			RCLCPP_ERROR(this->get_logger(), "Unable to open %s", port_.c_str());
			return false;
		}
		
		RCLCPP_INFO(this->get_logger(), "%s opened successfully.", port_.c_str());

		struct termios tio;
		tcgetattr(serial_fd_, &tio);
		cfmakeraw(&tio);
		tio.c_cflag = CS8 | CLOCAL | CREAD;
		tio.c_iflag &= ~(IXON | IXOFF);
		cfsetspeed(&tio, SERIAL_SPEED);
		tio.c_cc[VTIME] = 0;
		tio.c_cc[VMIN] = 0;

		if (tcsetattr(serial_fd_, TCSAFLUSH, &tio) != 0) 
		{
			RCLCPP_ERROR(this->get_logger(), "tcsetattr() failed.");
			close(serial_fd_);
			serial_fd_ = -1;
			return false;
		}
		return true;
	}

	int send_and_receive(const char* command, double* returned_data, int data_length)
	{
		char temp_buff[256];
		read(serial_fd_, temp_buff, 256); // Clear buffer

		if (write(serial_fd_, command, strlen(command)) < 0) {
			RCLCPP_WARN(this->get_logger(), "Failed to write to serial port.");
			return -1;
		}

		const int buff_size = 1024;
		char recv_buff[buff_size + 1];
		int recv_len = 0;
		auto start_time = this->now();

		while (recv_len < buff_size) 
		{
			int n = read(serial_fd_, recv_buff + recv_len, buff_size - recv_len);
			if (n < 0) {
				RCLCPP_WARN(this->get_logger(), "Failed to read from serial port.");
				return -1;
			}
			if (n > 0) 
			{
				recv_len += n;
				if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') {
					break;
				}
			}

			if ((this->now() - start_time).seconds() * 1000 > COMM_RECV_TIMEOUT_MS) {
				break; // Timeout
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		recv_buff[recv_len] = '\0';

		if (recv_len > 0 && recv_buff[0] == '!') {
			return -1; // Error from device
		}

		if (strncmp(command, recv_buff, strlen(command) - 1) == 0 && recv_buff[strlen(command) - 1] == '=') 
		{
			int data_count = 0;
			char* p = &recv_buff[strlen(command)];
			char* pp = nullptr;

			for (int i = 0; i < data_length; i++) 
			{
				if (p[0] == '0' && p[1] == 'x') {
					returned_data[i] = strtol(p + 2, &pp, 16);
				} else {
					returned_data[i] = strtod(p, &pp);
				}
				data_count++;

				if (*pp == ',') {
					p = pp + 1;
				} else {
					break;
				}
			}
			return data_count;
		}
		return 0;
	}

	bool reset_device_euler_angles()
	{
		double data_buffer[10];
		return send_and_receive("ra\n", data_buffer, 10) >= 0;
	}

	void process_imu_data()
	{
		if (serial_fd_ < 0) return;

		const int max_data = 10;
		double data[max_data];

		if (send_and_receive("g\n", data, max_data) >= 3) 
		{
			imu_data_.angular_velocity_x = data[0] * (M_PI / 180.0);
			imu_data_.angular_velocity_y = data[1] * (M_PI / 180.0);
			imu_data_.angular_velocity_z = data[2] * (M_PI / 180.0);
		}

		if (send_and_receive("a\n", data, max_data) >= 3) 
		{
			const double GRAVITY = 9.80665;
			imu_data_.linear_acceleration_x = data[0] * GRAVITY;
			imu_data_.linear_acceleration_y = data[1] * GRAVITY;
			imu_data_.linear_acceleration_z = data[2] * GRAVITY;
		}

		if (send_and_receive("e\n", data, max_data) >= 3) 
		{
			imu_data_.roll  = data[0] * (M_PI / 180.0);
			imu_data_.pitch = data[1] * (M_PI / 180.0);
			imu_data_.yaw	= data[2] * (M_PI / 180.0);
		}

		publish_imu_message();
		
		if (publish_tf_) {
			publish_tf();
		}
	}

	void publish_imu_message()
	{
		tf2::Quaternion q;
		q.setRPY(imu_data_.roll, imu_data_.pitch, imu_data_.yaw);

		imu_msg_.header.stamp = this->now();
		imu_msg_.header.frame_id = "imu_link";

		imu_msg_.orientation.x = q.x();
		imu_msg_.orientation.y = q.y();
		imu_msg_.orientation.z = q.z();
		imu_msg_.orientation.w = q.w();

		imu_msg_.angular_velocity.x = imu_data_.angular_velocity_x;
		imu_msg_.angular_velocity.y = imu_data_.angular_velocity_y;
		imu_msg_.angular_velocity.z = imu_data_.angular_velocity_z;

		imu_msg_.linear_acceleration.x = imu_data_.linear_acceleration_x;
		imu_msg_.linear_acceleration.y = imu_data_.linear_acceleration_y;
		imu_msg_.linear_acceleration.z = imu_data_.linear_acceleration_z;

		imu_pub_->publish(imu_msg_);
	}

	void publish_tf()
	{
		tf2::Quaternion q;
		q.setRPY(imu_data_.roll, imu_data_.pitch, imu_data_.yaw);

		geometry_msgs::msg::TransformStamped transform;
		transform.header.stamp = this->now();
		transform.header.frame_id = parent_frame_id_;
		transform.child_frame_id = "imu_link";
		
		transform.transform.translation.x = tf_pos_x_;
		transform.transform.translation.y = tf_pos_y_;
		transform.transform.translation.z = tf_pos_z_;

		transform.transform.rotation.x = q.x();
		transform.transform.rotation.y = q.y();
		transform.transform.rotation.z = q.z();
		transform.transform.rotation.w = q.w();

		tf_broadcaster_->sendTransform(transform);
	}

	void set_covariance_values()
	{
		imu_msg_.linear_acceleration_covariance[0] = 0.0064;
		imu_msg_.linear_acceleration_covariance[4] = 0.0063;
		imu_msg_.linear_acceleration_covariance[8] = 0.0064;
		imu_msg_.angular_velocity_covariance[0] = 0.032 * (M_PI / 180.0);
		imu_msg_.angular_velocity_covariance[4] = 0.028 * (M_PI / 180.0);
		imu_msg_.angular_velocity_covariance[8] = 0.006 * (M_PI / 180.0);
		imu_msg_.orientation_covariance[0] = 0.013 * (M_PI / 180.0);
		imu_msg_.orientation_covariance[4] = 0.011 * (M_PI / 180.0);
		imu_msg_.orientation_covariance[8] = 0.006 * (M_PI / 180.0);
	}

	void print_ascii_art()
	{
		std::cout << "                       | Z axis " << std::endl;
		std::cout << "                       | " << std::endl;
		std::cout << "                       |   / X axis " << std::endl;
		std::cout << "                   ____|__/____ " << std::endl;
		std::cout << "      Y axis     / *   | /    /| " << std::endl;
		std::cout << "      _________ /______|/    // " << std::endl;
		std::cout << "               /___________ // " << std::endl;
		std::cout << "              |____iahrs___|/ " << std::endl;
	}

	// Member variables with trailing underscore
	std::string port_;
	std::string parent_frame_id_;
	bool publish_tf_ = true;
	double tf_pos_x_, tf_pos_y_, tf_pos_z_;
	int serial_fd_ = -1;

	ImuData imu_data_;
	sensor_msgs::msg::Imu imu_msg_;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr euler_angle_reset_service_;
};

} // namespace iahrs_driver

// Moved signal handler out of the main logic, it's a global concern.
std::atomic<bool> g_stop_requested(false);
void signal_handler(int signal) 
{
	if (signal == SIGINT) {
    	g_stop_requested = true;
	}
}

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	
	auto node = std::make_shared<iahrs_driver::IahrsDriver>();
	node->run();

	rclcpp::shutdown();
    return 0;
}
