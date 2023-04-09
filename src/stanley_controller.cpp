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

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

# define PI           3.14159265358979323846

class Stanley : public rclcpp::Node
{
    float vx, t_ttc, m, c, atan_m_, sec_m_;
	float L, steering_raw, speed;
	std::vector<float> axs, range;
	std::vector<int> btns;
    bool enableController;

	VectorXf r;
	VectorXf r_dot;
	VectorXf cosBeta;

    public:

    Stanley() : Node("stanley_control")
    {
        declare_parameter("k_gain", 0.5);

		drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [this](sensor_msgs::msg::Joy::SharedPtr msg){ process_joystick(msg);} );

        amcl_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/pf/pose/odom", 1, [this](nav_msgs::msg::Odometry::SharedPtr msg){ process_amcl(msg);} ) ;

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ process_laser(msg);} );

        timer_ = this->create_wall_timer(25ms, [this]{ timer_callback(); });

        cosBeta = VectorXf::LinSpaced(1081, -3*PI/4, 3*PI/4).array().cos();

        m = -0.03202898550724638;

        c = 0.07985507246376813;

        atan_m_ = atan(m);

        sec_m_ = sqrt(1 + m*m);

        speed = 1.0;

        vx = speed;

		steering_raw = 0.0;

        enableController = false;
    }

    private:

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
	}

	void process_laser(const sensor_msgs::msg::LaserScan::SharedPtr laser_in)
	{
		range = laser_in.get()->ranges;
		int n = range.size();
		float* range_ptr = &range[0];
		Map<ArrayXf> r(range_ptr, n);
		ArrayXf t_all = r.segment(540-20, 40) / (vx*cosBeta.segment(540-20, 40).array()).cwiseMax(0.00f);
		t_ttc = t_all.minCoeff();
		if(t_ttc <= 0.75)
		{
			enableController = false;
		}
	}

	void process_amcl(const nav_msgs::msg::Odometry::SharedPtr odom_in)
	{
        float k = get_parameter("k_gain").as_double();

		vx = odom_in->twist.twist.linear.x;

        float angle = 2*atan2(odom_in->pose.pose.orientation.z, odom_in->pose.pose.orientation.w);
        float x = odom_in->pose.pose.position.x;
        float y = odom_in->pose.pose.position.y;

        float e = (m*x - y + c) / sec_m_;

        steering_raw = atan_m_ - angle + atan(k*e/vx);
	}

	void timer_callback()
	{
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;

		drive_msg.drive.steering_angle = steering_raw*int(enableController);
		drive_msg.drive.speed = speed*int(enableController);

		drive_pub_->publish(drive_msg);
	}

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr amcl_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stanley>());
  rclcpp::shutdown();
  return 0;
}
