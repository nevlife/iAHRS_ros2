#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <csignal>
#include <atomic>
#include <thread>
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

static const speed_t SERIAL_SPEED = B115200;
static const int COMM_RECV_TIMEOUT_MS = 30;

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

class IahrsDriver : public rclcpp::Node
{
public:
	IahrsDriver() : Node("iahrs_driver_async")
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
		imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("iahrs_imu/data", 10);
		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_euler_angles",
      std::bind(
        &IahrsDriver::reset_euler_angles_callback, this, std::placeholders::_1,
        std::placeholders::_2));

		set_covariance_values();

		RCLCPP_INFO(get_logger(), "IahrsDriver node has been initialized.");
	}

	~IahrsDriver()
	{
		// Signal the thread to stop and wait for it to finish
		if (serial_thread_.joinable()) {
			stop_thread_ = true;
			serial_thread_.join();
		}
		if (serial_fd_ >= 0) {
			close(serial_fd_);
		}
	}

	void run()
	{
		if (!open_serial_port()) {
			RCLCPP_FATAL(this->get_logger(), "Failed to open serial port. Shutting down.");
			return;
		}

		reset_device_euler_angles();
		RCLCPP_INFO(this->get_logger(), "Device Euler angles have been reset.");
		
		print_ascii_art();

		// Start the serial reading thread
		serial_thread_ = std::thread(&IahrsDriver::serial_read_thread, this);
	}

private:
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

	void serial_read_thread()
	{
		RCLCPP_INFO(this->get_logger(), "Serial read thread started.");
		while (rclcpp::ok() && !stop_thread_) {
			process_imu_data();
			// Add a small delay to prevent busy-waiting and high CPU usage
			std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Corresponds to 100Hz
		}
		RCLCPP_INFO(this->get_logger(), "Serial read thread finished.");
	}

	int send_and_receive(const char* command, double* returned_data, int data_length)
	{
		// Simple mutex to prevent concurrent access to the serial port
		std::lock_guard<std::mutex> lock(serial_mutex_);

		char temp_buff[256];
		read(serial_fd_, temp_buff, 256); // Clear buffer

		if (write(serial_fd_, command, strlen(command)) < 0) {
			RCLCPP_WARN(this->get_logger(), "Failed to write to serial port.");
			return -1;
		}

		const int buff_size = 1024;
		char recv_buff[buff_size + 1];
		int recv_len = 0;
		auto start_time = std::chrono::steady_clock::now();

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

			auto now = std::chrono::steady_clock::now();
			if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > COMM_RECV_TIMEOUT_MS) {
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

	void reset_euler_angles_callback(
		const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
		std::shared_ptr<std_srvs::srv::Trigger::Response> response)
	{
		(void)request; // Mark as unused

		std::lock_guard<std::mutex> lock(serial_mutex_);
		const char * reset_command = "$sc,rst\r\n";
		if (write(serial_fd_, reset_command, strlen(reset_command)) > 0) {
			RCLCPP_INFO(get_logger(), "Sent Euler angle reset command to the sensor.");
			response->success = true;
			response->message = "Euler angles reset successfully.";
		} else {
			RCLCPP_ERROR(get_logger(), "Failed to send reset command.");
			response->success = false;
			response->message = "Failed to send reset command.";
		}
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

		auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
		
		imu_msg->header.stamp = this->now();
		imu_msg->header.frame_id = "imu_link";

		imu_msg->orientation.x = q.x();
		imu_msg->orientation.y = q.y();
		imu_msg->orientation.z = q.z();
		imu_msg->orientation.w = q.w();

		imu_msg->angular_velocity.x = imu_data_.angular_velocity_x;
		imu_msg->angular_velocity.y = imu_data_.angular_velocity_y;
		imu_msg->angular_velocity.z = imu_data_.angular_velocity_z;

		imu_msg->linear_acceleration.x = imu_data_.linear_acceleration_x;
		imu_msg->linear_acceleration.y = imu_data_.linear_acceleration_y;
		imu_msg->linear_acceleration.z = imu_data_.linear_acceleration_z;

		// Copy covariance values
		imu_msg->orientation_covariance = orientation_covariance_;
		imu_msg->angular_velocity_covariance = angular_velocity_covariance_;
		imu_msg->linear_acceleration_covariance = linear_acceleration_covariance_;

		imu_publisher_->publish(std::move(imu_msg));
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
		// These could be parameters as well
		linear_acceleration_covariance_[0] = 0.0064;
		linear_acceleration_covariance_[4] = 0.0063;
		linear_acceleration_covariance_[8] = 0.0064;
		angular_velocity_covariance_[0] = 0.032 * (M_PI / 180.0);
		angular_velocity_covariance_[4] = 0.028 * (M_PI / 180.0);
		angular_velocity_covariance_[8] = 0.006 * (M_PI / 180.0);
		orientation_covariance_[0] = 0.013 * (M_PI / 180.0);
		orientation_covariance_[4] = 0.011 * (M_PI / 180.0);
		orientation_covariance_[8] = 0.006 * (M_PI / 180.0);
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

	// Member variables
	std::string port_;
	std::string parent_frame_id_;
	bool publish_tf_ = true;
	double tf_pos_x_, tf_pos_y_, tf_pos_z_;
	int serial_fd_ = -1;

	ImuData imu_data_;
	
	// Covariance arrays
	std::array<double, 9> orientation_covariance_{};
	std::array<double, 9> angular_velocity_covariance_{};
	std::array<double, 9> linear_acceleration_covariance_{};

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

	// Threading and synchronization
	std::thread serial_thread_;
	std::atomic<bool> stop_thread_{false};
	std::mutex serial_mutex_;
};

} // namespace iahrs_driver

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	
	auto node = std::make_shared<iahrs_driver::IahrsDriver>();
	node->run();

	// Spin the node to process callbacks (e.g., for services)
	rclcpp::spin(node);

	rclcpp::shutdown();
    return 0;
}
