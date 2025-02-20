#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "payloadSdkInterface.h"
#include <curl/curl.h>
#include <filesystem>
#include <regex>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

// 定义连接参数
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};

class EOImageNode : public rclcpp::Node {
public:
    EOImageNode() : Node("eo_image_node") {
        // 声明并获取参数
        this->declare_parameter("save_path", "src/eo_image/assets");
        save_path_ = this->get_parameter("save_path").as_string();
        std::filesystem::create_directories(save_path_);
        
        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("eo_image", 10);

        // 初始化SDK和CURL
        initializePayloadSDK();
        initializeCURL();

        // 创建单个10Hz定时器来处理所有操作
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&EOImageNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "EOImageNode initialized at 10Hz");
        RCLCPP_INFO(this->get_logger(), "Saving latest image to: %s", save_path_.c_str());
    }

    ~EOImageNode() {
        if (curl_) {
            curl_easy_cleanup(curl_);
            curl_global_cleanup();
        }
        if (my_payload_) {
            my_payload_->sdkQuit();
            delete my_payload_;
        }
    }

private:
    void initializePayloadSDK() {
        try {
            my_payload_ = new PayloadSdkInterface(s_conn);
            my_payload_->sdkInitConnection();
            my_payload_->checkPayloadConnection();
            
            // 设置为视频模式用于测试
            my_payload_->setPayloadCameraMode(CAMERA_MODE_VIDEO);
            
            // 设置相机记录源为EO
            my_payload_->setPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC, 
                                             PAYLOAD_CAMERA_RECORD_EO, 
                                             PARAM_TYPE_UINT32);
            
            // 等待模式切换
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 切换回图像模式
            my_payload_->setPayloadCameraMode(CAMERA_MODE_IMAGE);
            
            my_payload_->regPayloadStatusChanged(
                std::bind(&EOImageNode::onPayloadStatusChanged, this, 
                std::placeholders::_1, std::placeholders::_2));
            
            capture_state_ = CHECK_STORAGE;
            RCLCPP_INFO(this->get_logger(), "PayloadSDK initialized in EO mode");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize PayloadSDK: %s", e.what());
            throw;
        }
    }

    void initializeCURL() {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_ = curl_easy_init();
        if (!curl_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
            throw std::runtime_error("CURL initialization failed");
        }
    }

    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
        size_t totalSize = size * nmemb;
        response->append(static_cast<char*>(contents), totalSize);
        return totalSize;
    }

    static size_t write_data(void *ptr, size_t size, size_t nmemb, FILE *stream) {
        return fwrite(ptr, size, nmemb, stream);
    }

    void timer_callback() {
        if (!my_payload_) return;

        switch(capture_state_) {
            case CHECK_STORAGE:
                my_payload_->getPayloadStorage();
                break;
            case CHECK_CAPTURE_STATUS:
                my_payload_->getPayloadCaptureStatus();
                break;
            case CHECK_CAMERA_MODE:
                my_payload_->getPayloadCameraMode();
                break;
            case CHANGE_CAMERA_MODE:
                my_payload_->setPayloadCameraMode(CAMERA_MODE_IMAGE);
                capture_state_ = CHECK_CAMERA_MODE;
                break;
            case DO_CAPTURE:
                my_payload_->setPayloadCameraCaptureImage();
                capture_state_ = WAIT_CAPTURE_DONE;
                break;
            case WAIT_CAPTURE_DONE:
                my_payload_->getPayloadCaptureStatus();
                break;
            case DOWNLOADING:
                downloadAndPublishLatestImage();
                break;
            case IDLE:
                // 立即开始新的捕获循环
                capture_state_ = CHECK_STORAGE;
                break;
        }
    }

    void downloadAndPublishLatestImage() {
        std::string image_name = getLatestImageName();
        if (image_name.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No image found");
            capture_state_ = CHECK_STORAGE;  // 立即开始新的捕获
            return;
        }

        // 构建新的文件路径（始终使用相同的文件名）
        latest_image_path_ = save_path_ + "/latest_image.jpg";
        std::string url = "http://" + std::string(udp_ip_target) + ":8000/download/" + image_name;

        if (downloadImage(url, latest_image_path_)) {
            // 下载成功后立即读取并发布
            try {
                cv::Mat image = cv::imread(latest_image_path_);
                if (!image.empty()) {
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                    msg->header.stamp = this->now();
                    msg->header.frame_id = "camera";
                    publisher_->publish(*msg);
                    RCLCPP_DEBUG(this->get_logger(), "Image published successfully");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error publishing image: %s", e.what());
            }
        }
        
        // 立即开始新的捕获循环
        capture_state_ = CHECK_STORAGE;
    }

    std::string getLatestImageName() {
        std::string url = "http://" + std::string(udp_ip_target) + ":8000/list-file";
        std::string response;

        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);

        CURLcode res = curl_easy_perform(curl_);
        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get file list: %s", curl_easy_strerror(res));
            return "";
        }

        std::regex pattern("<a href=\"(/delete/.*?)\" class=\"delete-link\"");
        std::smatch match;
        std::string latest_image;
        int largest_num = -1;
        
        std::string::const_iterator searchStart(response.cbegin());
        while (std::regex_search(searchStart, response.cend(), match, pattern)) {
            if (match.size() > 1) {
                std::string imageURL = match[1];
                std::string imageName = imageURL.substr(imageURL.find_last_of('/') + 1);
                
                // 提取数字部分
                std::regex num_pattern("(\\d+)");
                std::smatch num_match;
                if (std::regex_search(imageName, num_match, num_pattern)) {
                    int current_num = std::stoi(num_match[1]);
                    if (current_num > largest_num) {
                        largest_num = current_num;
                        latest_image = imageName;
                    }
                }
            }
            searchStart = match.suffix().first;
        }

        if (!latest_image.empty()) {
            RCLCPP_INFO(this->get_logger(), "Found latest image (largest number): %s", latest_image.c_str());
        }

        return latest_image;
    }

    bool downloadImage(const std::string& url, const std::string& file_path) {
        FILE* fp = fopen(file_path.c_str(), "wb");
        if (!fp) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", file_path.c_str());
            return false;
        }

        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl_, CURLOPT_WRITEDATA, fp);

        CURLcode res = curl_easy_perform(curl_);
        fclose(fp);

        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to download image: %s", curl_easy_strerror(res));
            return false;
        }

        return true;
    }

    void onPayloadStatusChanged(int event, double* param) {
        switch(event) {
            case PAYLOAD_CAM_CAPTURE_STATUS: {
                handleCaptureStatus(param);
                break;
            }
            case PAYLOAD_CAM_STORAGE_INFO: {
                handleStorageInfo(param);
                break;
            }
            case PAYLOAD_CAM_SETTINGS: {
                handleCameraSettings(param);
                break;
            }
        }
    }

    void handleCaptureStatus(double* param) {
        if (capture_state_ == CHECK_CAPTURE_STATUS) {
            if (param[0] == 0) {
                capture_state_ = CHECK_CAMERA_MODE;
                RCLCPP_DEBUG(this->get_logger(), "Payload is idle, checking camera mode");
            } else {
                capture_state_ = CHECK_STORAGE;  // 直接尝试新的捕获
            }
        } else if (capture_state_ == WAIT_CAPTURE_DONE) {
            if (param[0] == 0) {
                capture_state_ = DOWNLOADING;
                RCLCPP_DEBUG(this->get_logger(), "Capture completed, starting download");
            }
        }
    }

    void handleStorageInfo(double* param) {
        if (capture_state_ == CHECK_STORAGE) {
            if (param[2] >= 10.0) {
                capture_state_ = CHECK_CAPTURE_STATUS;
                RCLCPP_DEBUG(this->get_logger(), "Storage ready (%.2f MB available)", param[2]);
            } else {
                RCLCPP_WARN(this->get_logger(), "Low storage space (%.2f MB)", param[2]);
                capture_state_ = CHECK_STORAGE;  // 继续尝试
            }
        }
    }

    void handleCameraSettings(double* param) {
        if (capture_state_ == CHECK_CAMERA_MODE) {
            if (param[0] == CAMERA_MODE_IMAGE) {
                capture_state_ = DO_CAPTURE;
                RCLCPP_DEBUG(this->get_logger(), "Camera in image mode, starting capture");
            } else {
                capture_state_ = CHANGE_CAMERA_MODE;
                RCLCPP_DEBUG(this->get_logger(), "Changing to image mode");
            }
        }
    }

    PayloadSdkInterface* my_payload_{nullptr};
    CURL* curl_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;  // 只使用一个定时器
    std::string save_path_;
    std::string latest_image_path_;
    enum CaptureState {
        IDLE = 0,
        CHECK_STORAGE,
        CHECK_CAPTURE_STATUS,
        CHECK_CAMERA_MODE,
        CHANGE_CAMERA_MODE,
        DO_CAPTURE,
        WAIT_CAPTURE_DONE,
        DOWNLOADING
    };
    CaptureState capture_state_{IDLE};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EOImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 