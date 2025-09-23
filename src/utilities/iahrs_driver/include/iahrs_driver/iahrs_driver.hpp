#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <utility>

#include <math.h>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <sstream>
#include <algorithm>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>

#include <Eigen/Geometry>

#define _USD_MATH_DEFINES
#define DEG2RAD M_PI/180.0
#define GRAVITY 9.80665

#define SERIAL_PORT     "/dev/sensors/iahrs"
#define SERIAL_BAUD     B115200     // speed_t
#define COMM_RECV_TIMEOUT   30
#define BUFFER_LENGTH       1024
#define TEMP_BUFFER_LENGTH  256
#define TIMER_FREQUENCY     20      // 20ms, 50hz
#define DATA_FREQUENCY      10

class iahrs_driver : public rclcpp::Node
{
    public:
        iahrs_driver();
        ~iahrs_driver();

    private:
        void init_param();
        void init_sensor();
        bool serial_connect();
        void close_connection();
        static unsigned long get_micro_seconds();
        void control(std::string command);

        void publish_iahrs(bool recv_imu, bool recv_mag, bool recv_temp);
        void publish_imu();
        void publish_mag();
        void publish_temp();

        void set_sync_port();
        void set_sync_period(const int period);
        void set_sync_config(const int config);

        void reset_iahrs();
        void empty_buffer();
        void init_data_msg();
        void reset_data_msg();

        void set_write_command(const std::string command, int arg=-1);
        std::string read_serial();
        std::vector<double> split_line(const std::string& input);

        Eigen::Quaterniond euler_to_quaternion(double roll, double pitch, double yaw);

        sensor_msgs::msg::Imu imu_msg;
        sensor_msgs::msg::MagneticField mag_msg;
        std_msgs::msg::Float64 temp_msg;
        std::string imu_topic = "iahrs/imu";
        std::string mag_topic = "iahrs/mag";
        std::string temp_topic = "iahrs/temp";

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub;
        std::unique_ptr<tf2_ros::TransformBroadcaster> iarhs_tf;

        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();

        int serial_fd = -1;     // file descriptor
        int max_connection_tried = 10;
        int sync_data_config = 0;
        const int data_size = 20;
        bool is_debug = true;
        bool flag_imu, flag_mag, flag_temp;

        int c_stmp,	c_temp,	c_racc, c_rgyr, c_rmag, c_gacc,
            c_eule, c_quat, c_qvel, c_qpos, c_frqm;
        std::vector<double> oc, avc, lac;

};
