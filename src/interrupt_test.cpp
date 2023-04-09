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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using namespace Eigen;

# define PI 3.14159265358979323846

class Timer
{
    bool clear = false;

public:
    template<typename Function>
    void setTimeout(Function function, int delay);

    void stop();
};

void Timer::setTimeout(auto function, int delay)
{
    this->clear = false;
    std::thread t([=]() {
        if(this->clear) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        if(this->clear) return;
        function();
    });
    t.detach();
}

void Timer::stop()
{
    this->clear = true;
}

class Interrupter : public rclcpp::Node
{
	public:

	key_t key = ftok("/home/f1tenth3/shmfile", 65);
	int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
    Timer t;
	
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
        t.setTimeout([&]() {cout << "Hello!!!" << endl;}, 4000);
        publisher_->publish(drive_msg);
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
