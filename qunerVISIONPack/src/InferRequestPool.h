#ifndef INFER_REQUEST_POOL_H
#define INFER_REQUEST_POOL_H

#include <openvino/openvino.hpp>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gary_msgs/msg/auto_aim.hpp"

class InferRequestPool {
public:
    InferRequestPool(ov::CompiledModel& compiledModel, size_t poolSize, rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr publisher);
    void submitInferRequest(const cv::Mat& img);
    void processResult(size_t index);

private:
    std::vector<ov::InferRequest> inferRequests_;
    std::queue<size_t> idleRequests_;
    std::mutex mutex_;
    std::condition_variable condition_;
    ov::CompiledModel& compiledModel;
    int img_width;
    int img_height;
    rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr publisher_;
};

#endif // INFER_REQUEST_POOL_H
