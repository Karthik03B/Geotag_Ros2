#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <fstream>
#include <string>
#include <chrono>
#include <filesystem>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber"), save(false) {
        subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageSubscriber::imageCallback1, this, std::placeholders::_1));
        
        navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ap/navsat/navsat0", 10, std::bind(&ImageSubscriber::navsatCallback, this, std::placeholders::_1));

        subscription2_ = this->create_subscription<std_msgs::msg::Int32>(
            "trigger/value", 10, std::bind(&ImageSubscriber::imageCallback2, this, std::placeholders::_1));
        images_path_ = "/home/linux/geotag_images/";
        images_meta_ = "/home/linux/geotag_images/meta.txt";
        count = Imagecount();
    }


private:
   int Imagecount() {
        int imageCount = 0;
        std::filesystem::path dir(images_path_);
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
                imageCount++;
            }
        }
        return imageCount;
    }
    void imageCallback1(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR_STREAM(get_logger(), "cv_bridge exception: " << e.what());
            return;
        }
        if(save){
         std::string image_path = images_path_ + "image_" + std::to_string(count) + ".jpg";
        cv::imwrite(image_path, cv_ptr->image);

        std::ofstream meta_file(images_meta_, std::ios::app);
        meta_file << "image_"<< "-" << std::to_string(count) << ".jpg," << get_timestamp() << std::endl;
        meta_file << "Altitude: " << altitude_ << "meters" << std::endl;
        meta_file << "Latitude: " << latitude_ << "degrees" << std::endl;
        meta_file << "Longitude: " << longitude_ << std::endl;
        count++;
         meta_file.close();
       

      RCLCPP_INFO(this->get_logger(), "Image saved: %s", image_path.c_str());
      save = false;
      }
    }

    void imageCallback2(const std_msgs::msg::Int32::SharedPtr msg) {
        if(msg->data == 1) {
            save = true;
        } else {
            save = false;
        }
    }
    
    void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        altitude_ = msg->altitude;
        latitude_ = msg->latitude;
        longitude_ = msg->longitude;
     }
     std::string get_timestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        char buf[20];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H:%M:%S", std::localtime(&now_c));
        return buf;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription2_;
    bool save;
    std::string images_path_;
    std::string images_meta_;
    int count;
    double altitude_;
    double latitude_;
    double longitude_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
