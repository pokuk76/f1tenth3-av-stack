#include <chrono>
#include <fstream>
#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "eiquadprog.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

# define PI           3.14159265358979323846

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

int k;

MatrixXf meshgrid(VectorXf x, VectorXf y)
{

    int n = x.size();
    int m = y.size();
    MatrixXf xx = x.transpose().replicate(m, 1);
    MatrixXf yy = y.replicate(1, n);

    MatrixXf M(2*m, n);

    M << xx, yy;

    return M;

}

class CSVData
{
    public:
    MatrixXf data;
    string filename;

    CSVData(string filename_, MatrixXf data_)
    {
        filename = filename_;
        data = data_;
    }

    void writeToCSVfile()
    {
        ofstream file(filename.c_str());
        file << data.format(CSVFormat);
        file.close();
    }

    MatrixXf readFromCSVfile()
    {
        vector<float> matrixEntries;
        ifstream matrixDataFile(filename);
        string matrixRowString;
        string matrixEntry;
        int matrixRowNumber = 0;
    
        while (getline(matrixDataFile, matrixRowString))
        {
            stringstream matrixRowStringStream(matrixRowString);
            while (getline(matrixRowStringStream, matrixEntry, ','))
            {
                matrixEntries.push_back(stod(matrixEntry));
            }
            matrixRowNumber++;
        }
        
        return Map<Matrix<float, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
    }
};

class SafeProbability
{
    public:
    _2D::BicubicInterpolator<float> interp;

    float b, epsilon, alpha;

    MatrixXf A = MatrixXf::Zero(2, 2);
    MatrixXf B = MatrixXf::Zero(2, 1);

    float LfF;
    MatrixXf LgF;
    MatrixXd Q, Ge, Gi;
    VectorXd u0, he, hi, u; 

    SafeProbability()
    {
        b = 1e-3;
        A << 0, 1,
             0, -b;
        B << 0, 1;
        CSVData csv("/home/f1tenth3/f1tenth_ws/src/av-stack/data/case" + to_string(k+1) + "_filt.csv", MatrixXf::Zero(1, 1));
        cout << "File: " << "/home/f1tenth3/f1tenth_ws/src/av-stack/data/case" + to_string(k+1) + "_filt.csv" <<  endl;
        MatrixXf F = csv.readFromCSVfile();
        int nx = F.cols();
        int nv = F.rows();
        MatrixXf M = meshgrid(VectorXf::LinSpaced(nx, 0, 100), VectorXf::LinSpaced(nv, 0, 5));
        MatrixXf xx = M.block(0, 0, nv, nx);
        MatrixXf vv = M.block(nv, 0, nv, nx);
        interp.setData( xx, vv, F );
    }

    VectorXf solveQP(VectorXf u0_, MatrixXf Gi_, VectorXf hi_)
    {
        u0 = -u0_.cast <double> ();
        Gi = (Gi_.transpose()).cast <double> ();
        hi = hi_.cast <double> ();
        Q = MatrixXd::Identity(u0.rows(), u0.rows());
        Ge.resize(Q.rows(), 0);
        he.resize(0);
        solve_quadprog(Q, u0, Ge, he, Gi, hi, u);
        VectorXf u_f = u.cast <float> ();
        return u_f;
    }

    float F(VectorXf x)
    {   
        float F_calc = ( (x(0) >= 0) & (x(0) <= 100) & (x(1) >= 0) & (x(1) <= 5) ) ? interp(x(0), x(1)) : 1.00;
        return (F_calc <= 1) ? F_calc : 1.00;
    }

    VectorXf gradF(VectorXf x)
    {
        int n = x.size();

        VectorXf dF = VectorXf::Zero(n);

        MatrixXf eye = MatrixXf::Identity(n, n);
        
        eye(0, 0) = 1.00; eye(1, 1) = 0.01;

        for (int j = 0; j < n; j++)
        {
            dF(j, 0) = ( F(x + eye.col(j)) - F(x - eye.col(j)) ) / (2*eye(j, j));
        }

        return dF;
    }

    void calcLF(VectorXf x)
    {
        VectorXf dF = gradF(x);
        LfF = (A * x).dot(dF);
        LgF = dF.transpose() * B;
    }

    float safeCtrl(VectorXf x, float uN)
    {
        VectorXf uN_ = VectorXf::Zero(1);
        uN_ << uN;
        calcLF(x);
        MatrixXf Gi = LgF;
        VectorXf hi(1); hi << LfF + alpha*(F(x) - (1 - epsilon));
        VectorXf u = solveQP(uN_, Gi, hi);
        return u(0);
    }

};

class SafeControl : public rclcpp::Node
{
    float vx, ak, vk, t_ttc, m, c, atan_m_, sec_m_, Kp, x_offset;
	float L, steering_raw, speed;
	std::vector<float> axs, range;
	std::vector<int> btns;
    bool enableController;
    bool enableSafeControl;

	VectorXf r;
	VectorXf r_dot;
	VectorXf cosBeta;
    VectorXf x_;

    SafeProbability sp;

    public:

    SafeControl() : Node("safe_control")
    {
        declare_parameter("k_gain", 0.5);
		declare_parameter("speed", 1.0);
        declare_parameter("slope", 0.0);
		declare_parameter("intercept", -0.73);
        declare_parameter("alpha", 0.30);
        declare_parameter("epsilon", 0.30);
        declare_parameter("Kp", 0.10);
        declare_parameter("index", 3);
        declare_parameter("switch_time", 0.00);
        // declare_parameter("offset", 0);
        
		// drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

        diag_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/test_array", 1);

        // joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [this](sensor_msgs::msg::Joy::SharedPtr msg){ process_joystick(msg);} );

        // amcl_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/pf/pose/odom", 1, [this](nav_msgs::msg::Odometry::SharedPtr msg){ process_amcl(msg);} ) ;

        // laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ process_laser(msg);} );

        timer_ = this->create_wall_timer(12.5ms, [this]{ timer_callback(); });

        cosBeta = VectorXf::LinSpaced(1081, -3*PI/4, 3*PI/4).array().cos();

        m = get_parameter("slope").as_double();

        k = get_parameter("index").as_int();

        c = get_parameter("intercept").as_double();

        // x_offset = get_parameter("offset").as_double();

        x_offset = -1*get_parameter("switch_time").as_double();

        Kp = get_parameter("Kp").as_double();

        sp.alpha = get_parameter("alpha").as_double();

        sp.epsilon = get_parameter("epsilon").as_double();

        atan_m_ = atan(m);

        sec_m_ = sqrt(1 + m*m);

        speed = get_parameter("speed").as_double();

        vx = speed;

        vk = speed;

        ak = 0;

		steering_raw = 0.0;

        x_ = VectorXf::Zero(2);

        enableController = false;

        enableSafeControl = false;
    }

    private:

	// void process_joystick(const sensor_msgs::msg::Joy::SharedPtr joy_in)
	// {
	// 	axs = joy_in.get()->axes;
	// 	btns = joy_in.get()->buttons;

	// 	if (btns[0] == 1)
	// 	{
	// 		enableController = true;
	// 	}
	// 	if (btns[1] == 1)
	// 	{
	// 		enableController = false;
	// 	}
	// 	if (btns[2] == 1)
	// 	{
	// 		enableSafeControl = true;
	// 	}
	// 	if (btns[3] == 1)
	// 	{
	// 		enableSafeControl = false;
    //         vk = 0;
	// 	}
	// }

	// void process_laser(const sensor_msgs::msg::LaserScan::SharedPtr laser_in)
	// {
	// 	range = laser_in.get()->ranges;
	// 	int n = range.size();
	// 	float* range_ptr = &range[0];
	// 	Map<ArrayXf> r(range_ptr, n);
	// 	ArrayXf t_all = r.segment(540-20, 40) / (vx*cosBeta.segment(540-20, 40).array()).cwiseMax(0.00f);
	// 	t_ttc = t_all.minCoeff();
	// 	if(t_ttc <= 0.75)
	// 	{
	// 		enableController = false;
    //         vk = 0;
	// 	}
	// }

	// void process_amcl(const nav_msgs::msg::Odometry::SharedPtr odom_in)
	// {
    //     float k_s = get_parameter("k_gain").as_double();

	// 	vx = odom_in->twist.twist.linear.x;

    //     float angle = 2*atan2(odom_in->pose.pose.orientation.z, odom_in->pose.pose.orientation.w);
    //     float x = odom_in->pose.pose.position.x;
    //     float y = odom_in->pose.pose.position.y;

    //     float e = (m*x - y + c) / sec_m_;

    //     steering_raw = atan_m_ - angle + atan(k_s*e/vx);

    //     x_ << x - x_offset, vx;
    //     ak = sp.safeCtrl(x_, Kp*(speed - vx))*int(enableSafeControl)*int(enableController);
    //     // ak = Kp*(speed - vx)*int(enableSafeControl);
    //     vk += 0.025*ak;

    //     std_msgs::msg::Float32MultiArray msg;

    //     msg.data.push_back(x);
    //     msg.data.push_back(vk);
    //     msg.data.push_back(ak);
    //     msg.data.push_back(sp.F(x_));

    //     diag_pub_->publish(msg);
	// }

	void timer_callback()
	{
        // ackermann_msgs::msg::AckermannDriveStamped drive_msg;

		// drive_msg.drive.steering_angle = steering_raw*int(enableController);
		// drive_msg.drive.speed = vk*int(enableController);

		// drive_pub_->publish(drive_msg);

        x_ << 40.00, 1.0;

        ak = sp.safeCtrl(x_, 0.00);

        std_msgs::msg::Float32MultiArray msg;

        msg.data.push_back(ak);

        diag_pub_->publish(msg);
	}

    // rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diag_pub_;
	// rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr amcl_sub_;
	// rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
	// rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafeControl>());
  rclcpp::shutdown();
  return 0;
}