#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std;
using namespace Eigen;

# define PI 3.14159265358979323846

class Interrupter : public rclcpp::Node
{
	public:

	key_t key = ftok("shmfile", 65);
	int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
	
	Interrupter() : Node("Interrupter")
	{
        subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/read_data", 1, [this](std_msgs::msg::Int32::SharedPtr msg){ msg_callback(msg); });
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/interrup_data", 1);
        // timer_ = this->create_wall_timer(100ms, [this]{ timer_callback(); });
	}

	private:

    void msg_callback(const std_msgs::msg::Int32::SharedPtr msg_in)
    {
        int data = msg_in.get()->data;
        auto drive_msg = std_msgs::msg::Int32();
        drive_msg.data = data;
        publisher_->publish(drive_msg);
        sleep(4);
    }
	
	// void timer_callback()
	// {
	//  auto drive_msg = std_msgs::msg::Int32();
	//  int *data = (int*) shmat(shmid, (void*)0, 0);
	//  drive_msg.data = *data;
	//  publisher_->publish(drive_msg);
	// }

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
	
	// rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Interrupter>());
  rclcpp::shutdown();
  return 0;
}
