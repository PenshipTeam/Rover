#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <torch/script.h> // PyTorch C++ API
#include <chrono>

class ObjectDetector : public rclcpp::Node {
public:
    ObjectDetector() : Node("object_detector") {
        // Load the trained model
        try {
            module_ = torch::jit::load("best1.pt");
        } catch (const c10::Error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading the model: %s", e.what());
            return;
        }

        // Subscribe to the camera image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ObjectDetector::image_callback, this, std::placeholders::_1));

        // Publisher for the processed image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/detected_image", 10);

        // Initialize FPS calculation
        fps_start_time_ = std::chrono::high_resolution_clock::now();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Preprocess the image for the model
        cv::Mat resized_image;
        cv::resize(cv_ptr->image, resized_image, cv::Size(640, 480)); // Resize to model input size
        torch::Tensor tensor_image = torch::from_blob(resized_image.data, {resized_image.rows, resized_image.cols, 3}, torch::kByte);
        tensor_image = tensor_image.permute({2, 0, 1}).to(torch::kFloat).div(255).unsqueeze(0);

        // Perform object detection
        auto output = module_.forward({tensor_image}).toTuple();
        auto detections = output->elements()[0].toTensor();

        // Draw bounding boxes and labels
        for (int i = 0; i < detections.size(0); i++) {
            float x1 = detections[i][0].item<float>();
            float y1 = detections[i][1].item<float>();
            float x2 = detections[i][2].item<float>();
            float y2 = detections[i][3].item<float>();
            float confidence = detections[i][4].item<float>();
            int class_id = detections[i][5].item<int>();

            if (confidence > 0.5) { // Confidence threshold
                cv::rectangle(cv_ptr->image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                cv::putText(cv_ptr->image, "Class " + std::to_string(class_id), cv::Point(x1, y1 - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Calculate and display FPS
        auto fps_end_time = std::chrono::high_resolution_clock::now();
        auto fps_duration = std::chrono::duration_cast<std::chrono::milliseconds>(fps_end_time - fps_start_time_);
        float fps = 1000.0 / fps_duration.count();
        cv::putText(cv_ptr->image, "FPS: " + std::to_string(fps), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        fps_start_time_ = fps_end_time;

        // Publish the processed image
        publisher_->publish(*cv_ptr->toImageMsg());
    }

    torch::jit::script::Module module_; // PyTorch model
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::chrono::high_resolution_clock::time_point fps_start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetector>());
    rclcpp::shutdown();
    return 0;
}