#include <iahrs_driver/iahrs_driver.hpp> // include?

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<iahrs_driver>());
    rclcpp::shutdown();

    return 0;
}