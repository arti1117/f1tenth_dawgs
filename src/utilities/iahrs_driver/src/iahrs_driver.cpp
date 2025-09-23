#include <iahrs_driver/iahrs_driver.hpp>

iahrs_driver::iahrs_driver()
: Node("iahrs_driver")
{
    // Load parameters

    // Publisher
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(1));
    mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, rclcpp::QoS(1));
    temp_pub = this->create_publisher<std_msgs::msg::Float64>(temp_topic, rclcpp::QoS(1));

	// Timer
	timer_ = this->create_wall_timer(
		std::chrono::milliseconds(TIMER_FREQUENCY),
		std::bind(&iahrs_driver::timer_callback, this)
	);

	// Service
	// euler_angle_reset_srv_ = create_service<interfaces::srv::ImuReset>(
    //    	"all_data_reset",
	// std::bind(&IAHRS::Euler_angle_reset_callback, this, std::placeholders::_1, std::placeholders::_2));

	// Constructing
	init_param();
	init_sensor();

}

iahrs_driver::~iahrs_driver() {
	close_connection();
}

void iahrs_driver::init_param(){
	RCLCPP_INFO(this->get_logger(), "Init parameter");

	this->declare_parameter("is_debug", is_debug);
	this->get_parameter("is_debug", is_debug);

	this->declare_parameter("imu_topic", imu_topic);
	this->declare_parameter("mag_topic", mag_topic);
	this->declare_parameter("temp_topic", temp_topic);

	this->declare_parameter<std::vector<double>>("imu_data.lin_acc_cov", std::vector<double> {});
	this->declare_parameter<std::vector<double>>("imu_data.ang_vel_cov", std::vector<double> {});
	this->declare_parameter<std::vector<double>>("imu_data.orient_cov", std::vector<double> {});

	this->declare_parameter<int>("object.sync_data_type.stmp", 0x0001);
	this->declare_parameter<int>("object.sync_data_type.temp", 0x0002);
	this->declare_parameter<int>("object.sync_data_type.racc", 0x0004);
	this->declare_parameter<int>("object.sync_data_type.rgyr", 0x0008);
	this->declare_parameter<int>("object.sync_data_type.rmag", 0x0010);
	this->declare_parameter<int>("object.sync_data_type.gacc", 0x0020);
	this->declare_parameter<int>("object.sync_data_type.eule", 0x0040);
	this->declare_parameter<int>("object.sync_data_type.quat", 0x0080);
	this->declare_parameter<int>("object.sync_data_type.qvel", 0x0100);
	this->declare_parameter<int>("object.sync_data_type.qpos", 0x0200);
	this->declare_parameter<int>("object.sync_data_type.frqm", 0x0400);

	c_stmp = this->get_parameter("object.sync_data_type.temp").as_int();
	c_temp = this->get_parameter("object.sync_data_type.temp").as_int();
	c_racc = this->get_parameter("object.sync_data_type.racc").as_int();
	c_rgyr = this->get_parameter("object.sync_data_type.rgyr").as_int();
	c_rmag = this->get_parameter("object.sync_data_type.rmag").as_int();
	c_gacc = this->get_parameter("object.sync_data_type.gacc").as_int();
	c_eule = this->get_parameter("object.sync_data_type.eule").as_int();
	c_quat = this->get_parameter("object.sync_data_type.quat").as_int();
	c_qvel = this->get_parameter("object.sync_data_type.qvel").as_int();
	c_qpos = this->get_parameter("object.sync_data_type.qpos").as_int();
	c_frqm = this->get_parameter("object.sync_data_type.frqm").as_int();

	sync_data_config = c_temp | c_racc | c_rgyr | c_rmag | c_eule | c_quat;

	this->get_parameter("imu_data.orient_cov", oc);
	this->get_parameter("imu_data.lin_acc_cov", lac);
	this->get_parameter("imu_data.ang_vel_cov", avc);

	if(is_debug)
	{
		RCLCPP_INFO(this->get_logger(), "param: oc = %.3lf, %.3lf, %.3lf",
			oc[0]*1000, oc[4]*1000, oc[8]*1000);
		RCLCPP_INFO(this->get_logger(), "param: lac = %.3lf, %.3lf, %.3lf",
			lac[0]*1000, lac[4]*1000, lac[8]*1000);
		RCLCPP_INFO(this->get_logger(), "param: avc = %.3lf, %.3lf, %.3lf",
			avc[0]*1000, avc[4]*1000, avc[8]*1000);
	}
}

void iahrs_driver::init_sensor(){
	RCLCPP_INFO(this->get_logger(), "Init sensor");
	double data[data_size];
	int no_data = 0;
	if(serial_connect())
	{
		// reset angle
		set_write_command("ra");
		usleep(1000);
		set_write_command("rp");
		usleep(1000);
		set_write_command("za");
		usleep(1000);
 		set_sync_port();
 		set_sync_period(DATA_FREQUENCY);
 		set_sync_config(sync_data_config);
		empty_buffer();
	}
}

void iahrs_driver::timer_callback(){
	rclcpp::Time now = this->get_clock()->now();
	std::string serial_line;
	std::vector<double> bd;
	serial_line = read_serial();

	bd = split_line(serial_line);

	if(is_debug)
	{
		RCLCPP_INFO(this->get_logger(), "Reading: %s", serial_line.c_str());
		if(bd.size() > 10)
		{
		RCLCPP_INFO(this->get_logger(),
			"Data: %d, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf",
			bd.size(), bd[0], bd[1], bd[2], bd[3], bd[4], bd[5], bd[6],
			bd[7], bd[8], bd[9]);
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "data size: %d", bd.size());
		}
	}

	if(bd.size() > 10)
	{
		RCLCPP_INFO_ONCE(this->get_logger(), "Reading: %s", serial_line.c_str());
		Eigen::Quaterniond q = euler_to_quaternion(
			bd[10] * DEG2RAD, bd[11] * DEG2RAD, bd[12] * DEG2RAD);

		temp_msg.data = bd[0];

		imu_msg.header.stamp = now;
		imu_msg.header.frame_id = "iahrs";
		imu_msg.orientation.x = q.x();		// q.x() or bd[14]
		imu_msg.orientation.y = q.y();		// q.y() or bd[15]
		imu_msg.orientation.z = q.z();		// q.z() or bd[16]
		imu_msg.orientation.w = q.w();		// q.w() or bd[13]
		imu_msg.angular_velocity.x = bd[4] * DEG2RAD;
		imu_msg.angular_velocity.y = bd[5] * DEG2RAD;
		imu_msg.angular_velocity.z = bd[6] * DEG2RAD;
		imu_msg.linear_acceleration.x = bd[1] * GRAVITY;
		imu_msg.linear_acceleration.y = bd[2] * GRAVITY;
		imu_msg.linear_acceleration.z = bd[3] * GRAVITY;

		mag_msg.header.stamp = now;
		mag_msg.header.frame_id = "iahrs";
		mag_msg.magnetic_field.x = bd[7];
		mag_msg.magnetic_field.y = bd[8];
		mag_msg.magnetic_field.z = bd[9];


		if(is_debug)
		{
			RCLCPP_INFO(this->get_logger(),
				"measured: %.3lf, %.3lf, %.3lf, %.3lf", bd[14], bd[15], bd[16], bd[13]);
			RCLCPP_INFO(this->get_logger(),
				"converted: %.3lf, %.3lf, %.3lf, %.3lf", q.x(), q.y(), q.z(), q.w());
		}

		publish_iahrs(true, true, true);
	}
}

Eigen::Quaterniond iahrs_driver::euler_to_quaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll,   Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw,     Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.normalized();
}



bool iahrs_driver::serial_connect(){
    bool is_connected = false;
    int connection_count = max_connection_tried;

    do{
		RCLCPP_INFO(this->get_logger(), "Try connecting to %s", SERIAL_PORT);
        serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY);
        --connection_count;
		usleep(1000);
    }while(serial_fd < 0 && connection_count > 0);
	if (connection_count <= 0){
		RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),"Connection failed: Timeout" << std::endl);
		return is_connected;
	}

    struct termios tio;
    tcgetattr(serial_fd, &tio);
    cfmakeraw(&tio);
	tio.c_cflag = SERIAL_BAUD|CS8|CLOCAL|CREAD;
  	tio.c_iflag &= ~(IXON | IXOFF);
  	// cfsetspeed(&tio, SERIAL_BAUD);
  	tio.c_cc[VTIME] = 0;
  	tio.c_cc[VMIN] = 0;

    int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
  	if (err != 0)
	{
    	RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),"Serial setting failed: Attribute write " << errno << std::endl);
    	close(serial_fd);
		serial_fd = -1;

    	return is_connected;
  	}

	is_connected = true;
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"Connection success to " << SERIAL_PORT <<std::endl);
    return  is_connected;
}

void iahrs_driver::close_connection(){
	if(serial_fd)
	{
		RCLCPP_INFO(this->get_logger(), "Closing connection: %s", SERIAL_PORT);
		close(serial_fd);
	}
}

unsigned long iahrs_driver::get_micro_seconds(){

    struct timespec time_stamp;
	clock_gettime(CLOCK_MONOTONIC, &time_stamp);

    return time_stamp.tv_sec*1000 + time_stamp.tv_nsec/1000000;
}


std::string iahrs_driver::read_serial()
{
	// RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3700, )
	// read(serial_fd, returning_data, )
    std::string result;
    char ch;
    while (read(serial_fd, &ch, 1) == 1) {
        if (ch == '\n') break;
        result += ch;
    }
    return result;
}

std::vector<double> iahrs_driver::split_line(const std::string& input) {
    std::vector<double> serial_data;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, ',')) {
        try {
            double value = std::stod(token);
            serial_data.push_back(value);
        } catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(this->get_logger(), "Split failed: invalid arguments: %s", e.what());
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(this->get_logger(), "Split failed: out of range: %s", e.what());
        }
    }
    return serial_data;
}


void iahrs_driver::reset_iahrs()
{
	set_write_command("rd");
}
void iahrs_driver::empty_buffer()
{
	// Empty buffer
	RCLCPP_INFO(this->get_logger(), "Empty buffers");
	char temp_buffer[TEMP_BUFFER_LENGTH];
	read(serial_fd, temp_buffer, TEMP_BUFFER_LENGTH);
	for(int i =0; i<strlen(temp_buffer); i++){
		std::cout << temp_buffer[i];
	}
	usleep(1000);
}
void iahrs_driver::reset_data_msg()
{
	set_write_command("ra");
	usleep(1000);
	set_write_command("rp");
	usleep(1000);
	set_write_command("za");
	usleep(1000);
}


void iahrs_driver::set_write_command(const std::string command, int arg)
{
	RCLCPP_INFO(this->get_logger(), "Write command: %s (%d)", command.c_str(), arg);
	std::string new_command;
	if(arg >=0)
	{
		new_command = command + "=" + std::to_string(arg) + "\n";
	}
	else
	{
		new_command = command + "\n";
	}
	const char* new_command_c = new_command.c_str();
	write(serial_fd, new_command_c, strlen(new_command_c));
}
void iahrs_driver::set_sync_port()
{
	set_write_command("so", 1);
}
void iahrs_driver::set_sync_period(const int period)
{
	set_write_command("sp", period);
}
void iahrs_driver::set_sync_config(const int config)
{
	set_write_command("sd", config);
}

void iahrs_driver::publish_iahrs(bool recv_imu, bool recv_mag, bool recv_temp){
    if (recv_imu){
        iahrs_driver::publish_imu();
    }
    if (recv_mag){
        iahrs_driver::publish_mag();
    }
    if (recv_temp){
        iahrs_driver::publish_temp();
    }
}

void iahrs_driver::publish_imu(){
    // sensor_msgs::msg::Imu imu_msg;
	RCLCPP_INFO_ONCE(this->get_logger(), "publish imu msg");

	imu_pub->publish(imu_msg);
}
void iahrs_driver::publish_mag(){
    // sensor_msgs::msg::MagneticField mag_msg;
	RCLCPP_INFO_ONCE(this->get_logger(), "publish mag msg");

	mag_pub->publish(mag_msg);
}
void iahrs_driver::publish_temp(){
    // std_msgs::msg::Float64 temp_msg;
	RCLCPP_INFO_ONCE(this->get_logger(), "publish temp msg");

	temp_pub->publish(temp_msg);
}