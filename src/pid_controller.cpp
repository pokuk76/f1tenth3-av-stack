#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "slam_toolbox/map_saver.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;
using namespace Eigen;

# define PI           3.14159265358979323846


class PID : public rclcpp::Node
{
	int idx_90, idx_theta, counter;
	float vx, t_ttc;
	float L, steering_raw, speed;
	float theta, a, b, cos_theta, sin_theta, alpha, y_d;
	float steering, d_steering, steering_last, max_steering_delta;
	std::vector<float> axs, range;
	std::vector<int> btns;
	bool enableController, countEnable;
	
	VectorXf r;
	VectorXf r_dot;
	VectorXf cosBeta;
	float v_next, v_curr, alpha_, beta_;
	
	
	public:
	
	PID() : Node("pid_control")
	{
		drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
		set_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wallfollow/pid/setpoint", 1);
		feed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wallfollow/pid/state", 1);
		enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/wallfollow/pid/enable", 1);
		effort_sub_ = this->create_subscription<std_msgs::msg::Float64>("/wallfollow/pid/control_effort", 1, std::bind(&PID::get_control, this, _1));
		joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&PID::process_joystick, this, _1));
		laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&PID::process_laser, this, _1));
		amcl_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/vesc/odom", 1, std::bind(&PID::process_amcl, this, _1));
		client_ = create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");
		timer_ = this->create_wall_timer(10ms, std::bind(&PID::timer_callback, this));
		theta = 35.0;
		idx_90 = 180;
		L = 0.1;
		speed = 2.5;
		v_curr = 2.5;
		idx_theta = idx_90 + int(4*theta);
		cos_theta = cos(theta/180.00 * PI);
		sin_theta = sin(theta/180.00 * PI);
		cosBeta = VectorXf::LinSpaced(1081, -3*PI/4, 3*PI/4).array().cos();
		enableController = false;
		countEnable = false;
		steering = 0.00;
		steering_last = 0.00;
		max_steering_delta = 6.00/100.00;
		alpha_ = 0.1;
		beta_ = 0.1;
		counter = 0;
	}
	
	private:
	
	float f(float x)
	{
		return 2*(1 / (1 + pow(abs(x/0.2), 2*4)) - 1/2);
	}
	
	void set_speed(float angle)
	{
		v_next = speed*(1 - alpha_) + beta_*f(angle);
		speed = v_next;
		cout << speed << endl;
	}
	
	
	

	void timer_callback()
	{
		auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
		auto set_msg = std_msgs::msg::Float64();
		auto feed_msg = std_msgs::msg::Float64();
		auto enable_msg = std_msgs::msg::Bool();
		float speed_thr;
		alpha = atan((a*cos_theta - b)/(a*sin_theta));
		y_d = b*cos_theta + speed*sin(alpha);
		set_msg.data = 0.90;
		feed_msg.data = y_d;
		enable_msg.data = enableController;
		
		//d_steering = steering_raw - steering_last;
		//steering += std::max(std::min(d_steering, max_steering_delta), -max_steering_delta);
		//steering_last = steering;
		float norm_angle = steering_raw/(PI/180*19.5);
		//float norm_angle
		//float speed_thr = set_speed(steering_raw);
		speed_thr = abs(3.0*(1 - norm_angle*norm_angle));
		//set_speed(steering_raw);
		if(countEnable)
		{
			counter++;
			speed_thr = -0.75;
			steering_raw = 0.0;
			
		}
		if(countEnable && counter >= 200)
		{
			counter = 0;
			countEnable = false;
		}
		drive_msg.drive.steering_angle = steering_raw*int(enableController);
		drive_msg.drive.speed = speed_thr*int(enableController);
		drive_pub_->publish(drive_msg);
		set_pub_->publish(set_msg);
		feed_pub_->publish(feed_msg);
		enable_pub_->publish(enable_msg);
	}
	
	void get_control(const std_msgs::msg::Float64::SharedPtr ctrl_in)
	{
		steering_raw = 1*float(ctrl_in.get()->data);
	}
	
	void process_laser(const sensor_msgs::msg::LaserScan::SharedPtr laser_in)
	{
		range = laser_in.get()->ranges;
		a = range[idx_theta];
		b = range[idx_90];
		int n = range.size();
		float* range_ptr = &range[0];
		Map<ArrayXf> r(range_ptr, n);
		ArrayXf t_all = r.segment(540-20, 40) / (vx*cosBeta.segment(540-20, 40).array()).cwiseMax(0.00f);
		t_ttc = t_all.minCoeff();
		if(t_ttc <= 0.75)
		{
			enableController = false;
			//countEnable = false;
			counter = 0;
		}
	}
	
	void process_joystick(const sensor_msgs::msg::Joy::SharedPtr joy_in)
	{
		axs = joy_in.get()->axes;
		btns = joy_in.get()->buttons;
		if (btns[0] == 1)
		{
			enableController = true;
		}
		if (btns[1] == 1)
		{
			enableController = false;
		}
		if (btns[3] == 1)
		{
			queue_async_request();
		}
	}

	void process_amcl(const nav_msgs::msg::Odometry::SharedPtr odom_in)
	{
		vx = odom_in->twist.twist.linear.x;
	}
	
	void queue_async_request()
	{
		auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
		request->name.data = "/home/f1tenth3/f1tenth_ws/src/av-stack/maps/savedMapPID";
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
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feed_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr effort_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr amcl_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PID>());
  rclcpp::shutdown();
  return 0;
}
