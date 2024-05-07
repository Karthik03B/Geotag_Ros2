#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher()
        : Node("camera_publisher"),
          camera_loc(), 
          publisher_name("/camera/image_raw"),
          count(0) {
        
        if (!cam.open(camera_loc)) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera!");
            return;
        }
        pub = create_publisher<sensor_msgs::msg::Image>(publisher_name, 10);
        timer = create_wall_timer(std::chrono::milliseconds(500 / fps), std::bind(&CameraPublisher::timerCallback, this));
        RCLCPP_INFO(get_logger(), "Camera Publisher node initialized.");
    }

private:
    void timerCallback() {
        cv::Mat src;
        cam.read(src);

        if (!src.empty()) {
            cv::flip(src, src, 1);
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", src).toImageMsg();
            pub->publish(*img_msg);
            RCLCPP_INFO(get_logger(), "Published camera image");
            count = (count + 1) % 10;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to capture camera image");
        }
    }
    cv::VideoCapture cam;
    const int camera_loc;
    const std::string publisher_name;
    const int fps = 30;
    size_t count;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
