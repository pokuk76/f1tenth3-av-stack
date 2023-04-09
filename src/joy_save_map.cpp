#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Core>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "slam_toolbox/map_saver.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyTeleop : public rclcpp::Node
{
	std::vector<float> axs = std::vector<float>(8, 0.0);
	std::vector<int> btns;
	
	public:
	
	JoyTeleop() : Node("joystick_teleop")
	{
		publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
		subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [this](sensor_msgs::msg::Joy::SharedPtr msg){ process_joystick(msg);} );
		timer_ = this->create_wall_timer(20ms, [this]{ timer_callback(); });
		client_ = create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");
	}
	
	private:
	
	void timer_callback()
	{
		auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
		drive_msg.drive.steering_angle = 0.50*axs.at(0);
		drive_msg.drive.speed = (2.25*(axs.at(2) - axs.at(5)))/2; 
		publisher_->publish(drive_msg);
	}
	
	void process_joystick(const sensor_msgs::msg::Joy::SharedPtr joy_in)
	{
		axs = joy_in.get()->axes;
		btns = joy_in.get()->buttons;
		if (btns[3] == 1)
		{
			queue_async_request();
		}
	}
	
	void queue_async_request()
	{
		auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
		request->name.data = "/home/f1tenth3/f1tenth_ws/src/av-stack/maps/recordedMap2";
    		using ServiceResponseFuture =
      		rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedFuture;
    		auto response_received_callback = [this](ServiceResponseFuture future)
    		{
        		auto result = future.get();
        		rclcpp::shutdown();
      		};
    	auto future_result = client_->async_send_request(request, response_received_callback);
	}
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
	rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
