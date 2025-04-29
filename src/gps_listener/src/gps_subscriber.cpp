#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;

class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber()
    : Node("gps_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "gps_data", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        double lat = msg->x;
        double lng = msg->y;
        double a = lat+lng;
        RCLCPP_INFO(this->get_logger(), "a=%.6f",
                    a);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
