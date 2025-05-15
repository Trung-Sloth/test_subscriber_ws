#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <curl/curl.h>
#include <sstream>

using std::placeholders::_1;
float lat_cur_ = 10.850822;
float lng_cur_ = 106.773318;
class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber() : Node("gps_subscriber")
    {
        lat_subs[0] = this->create_subscription<std_msgs::msg::Float32>(
            "lat1", 10, std::bind(&GPSSubscriber::lat_callback1, this, _1));
        lat_subs[1] = this->create_subscription<std_msgs::msg::Float32>(
            "lat2", 10, std::bind(&GPSSubscriber::lat_callback2, this, _1));
        lat_subs[2] = this->create_subscription<std_msgs::msg::Float32>(
            "lat3", 10, std::bind(&GPSSubscriber::lat_callback3, this, _1));

        lng_subs[0] = this->create_subscription<std_msgs::msg::Float32>(
            "lng1", 10, std::bind(&GPSSubscriber::lng_callback1, this, _1));
        lng_subs[1] = this->create_subscription<std_msgs::msg::Float32>(
            "lng2", 10, std::bind(&GPSSubscriber::lng_callback2, this, _1));
        lng_subs[2] = this->create_subscription<std_msgs::msg::Float32>(
            "lng3", 10, std::bind(&GPSSubscriber::lng_callback3, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&GPSSubscriber::send_to_firebase, this));
    }

private:
    void lat_callback1(const std_msgs::msg::Float32::SharedPtr msg) {
        double lat1 = msg->data;
        printf("Toa-do-1 => Lat1: %.6f\n", lat1);
    }
    void lat_callback2(const std_msgs::msg::Float32::SharedPtr msg) {
        double lat2 = msg->data;
        printf("Toa-do-2 => Lat2: %.6f\n", lat2);
    }
    void lat_callback3(const std_msgs::msg::Float32::SharedPtr msg) {
        double lat3 = msg->data;
        printf("Toa-do-3 => Lat3: %.6f\n", lat3);
    }

    void lng_callback1(const std_msgs::msg::Float32::SharedPtr msg) {
        double lng1 = msg->data;
        printf("Toa-do-1 => Lng1: %.6f\n", lng1);
    }
    void lng_callback2(const std_msgs::msg::Float32::SharedPtr msg) {
        double lng2 = msg->data;
        printf("Toa-do-2 => Lng2: %.6f\n", lng2);
    }
    void lng_callback3(const std_msgs::msg::Float32::SharedPtr msg) {
        double lng3 = msg->data;
        printf("Toa-do-3 => Lng3: %.6f\n\n", lng3);
    }


    void send_to_firebase()
    {
        lat_cur_ = lat_cur_ + 0.000001;
        lng_cur_ = lng_cur_ + 0.00001;

        CURL *curl;
        CURLcode res;
        curl = curl_easy_init();
        if (curl) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(6);
            ss << "{\"lat_cur\": " << lat_cur_ << ", \"lng_cur\": " << lng_cur_ << "}";
            std::string data = ss.str();
            std::string url = "https://map-1-b0eae-default-rtdb.asia-southeast1.firebasedatabase.app/Toa-do-hien-tai.json";

            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());

            res = curl_easy_perform(curl);
            if (res != CURLE_OK)
                RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
            else
                RCLCPP_INFO(this->get_logger(), "Sent current location to Firebase: lat=%.6f, lng=%.6f", lat_cur_, lng_cur_);

            curl_easy_cleanup(curl);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lat_subs[3];
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lng_subs[3];
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
