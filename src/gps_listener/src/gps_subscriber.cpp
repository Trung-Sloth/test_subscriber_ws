#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber()
    : Node("gps_subscriber")
    {
        lat_sub = this->create_subscription<std_msgs::msg::Float32>(
            "lat", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
        lng_sub = this->create_subscription<std_msgs::msg::Float32>(
            "lng", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        double lat = msg->lat;
        double lng = msg->lng;
        // double a = lat+lng;
        // RCLCPP_INFO(this->get_logger(), "Kinh do - lng = %.6f\nVi do - lat = %.6f",
        //             lng,lat);
        printf("Kinh do - lng = %.6f\nVi do - lat = %.6f\n\n",
                     lng,lat);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
