//Include CAMERA DRIVER
#include <qunerCAM.h>

//Include OPENVINO DRIVER
#include <qunerVINO.h>
#include <InferRequestPool.h>

//Include ROS DRIVER
#include <qunerROS.h>

using namespace cv;
using namespace dnn;
using namespace std;
using namespace chrono_literals;

void ProcessImage(cv::Mat& img, InferRequestPool& inferRequestPool);

int qunerexposureTime = 4800;
int quner_multi = 2;
float qunerGain = 16.00;

// 控制图像捕获线程的运行
mutex frame_mutex;
queue<cv::Mat> frame_queue;
bool g_bAcquisitionFlag = true; 

//原图像
cv::Mat latestImage;
std::mutex imageMutex; // 用于同步访问的互斥锁
cv::Mat processedImage; // 用于存储处理后的图像
//腐蚀后
std::mutex imageMutex2; // 用于同步访问的互斥锁
cv::Mat processedImage2; // 用于存储处理后的图像


//原始相机帧率计算器
std::chrono::steady_clock::time_point lastTime1 = std::chrono::steady_clock::now();
int frameCount1 = 0;


int img_width = 640;
int img_height = 512;

//ROS2 全局传参
double yaw_act;
double pitch_act;

Camera* Camera::instance = nullptr;

Camera::Camera() {
    instance = this;
}

Camera::~Camera() {
    close();
}

bool Camera::open() {
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t deviceNumber = 0;
    //重要，初始化相机，否则无法调用。
    status = GXInitLib();
    status = GXUpdateDeviceList(&deviceNumber, 1000);
    if (deviceNumber <= 0) {
        printf("找不到你这该死相机。\n");
        return false;
    }

    if (status != GX_STATUS_SUCCESS) {
        std::cerr << "垃圾相机列表更新失败。\n" << status << std::endl;
        return false;
    }
    if (deviceNumber == 0) {
        std::cerr << "沙壁，没接相机！\n" << std::endl;
        return false;
    }

    GX_OPEN_PARAM openParam;
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;
    openParam.openMode = GX_OPEN_INDEX;
    openParam.pszContent = "1";
    status = GXOpenDevice(&openParam, &hDevice);
    if (status != GX_STATUS_SUCCESS) {
        std::cerr << "打不开你这破相机，被占用了！\n " << status << std::endl;
        return false;
    }
    status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, qunerexposureTime);
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, qunerGain);
    return true;
}


bool Camera::close() {
    if (hDevice != nullptr) {
        GXCloseDevice(hDevice);
        hDevice = nullptr;
    }
    return true;
}

bool Camera::startCapture() {
    if (hDevice != nullptr) {
        GX_STATUS status = GXRegisterCaptureCallback(hDevice, nullptr, OnFrameCallbackFun);
        if (status != GX_STATUS_SUCCESS) {
            return false;
        }

        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        return status == GX_STATUS_SUCCESS;
    }
    return false;
}

bool Camera::stopCapture() {
    if (hDevice != nullptr) {
        GX_STATUS status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        GXUnregisterCaptureCallback(hDevice);
        return status == GX_STATUS_SUCCESS;
    }
    return false;
}

//ROS2 回调 yaw
void callback_yaw(const gary_msgs::msg::DualLoopPIDWithFilter::SharedPtr msg_callback_yaw)
{
    yaw_act = msg_callback_yaw->outer_feedback;
    //cout << "I am working" << endl;

    //RCLCPP_INFO(rclcpp::get_logger("vision_callback_yaw"), "Received yaw_act: %f", yaw_act);
}

//ROS2 回调 pitch
void callback_pitch(const gary_msgs::msg::DualLoopPIDWithFilter::SharedPtr msg_callback_pitch)
{
    pitch_act = msg_callback_pitch->outer_feedback;
    //cout << "I am working" << endl;

    //RCLCPP_INFO(rclcpp::get_logger("vision_callback_pitch"), "Received pitch_act: %f", yaw_act);
}

void GX_STDC Camera::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame) {
    if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
        // 创建一个OpenCV的Mat，用于存放从相机收到的原始BayerRG格式图像
        void* nonConstBuffer = const_cast<void*>(pFrame->pImgBuf);
        cv::Mat rawImage(pFrame->nHeight, pFrame->nWidth, CV_8UC1, nonConstBuffer);

        cv::Mat bgrImage;
        // 使用cv::cvtColor将BayerRG格式图像转换为BGR格式
        cv::cvtColor(rawImage, bgrImage, cv::COLOR_BayerRG2BGR);

        // 设置目标图像大小
        cv::Size targetSize(img_width, img_height);
        cv::Mat resizedImage;

        // 缩放图像到目标大小
        cv::resize(bgrImage, resizedImage, targetSize);
/*
        frameCount1++; // 增加原始相机帧数计数器
        auto now1 = std::chrono::steady_clock::now(); // 获取当前时间
        double elapsedSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(now1 - lastTime1).count();

        if (elapsedSeconds >= 1.0) { // 如果自上次更新以来已经过了至少一秒
            double fps1 = frameCount1 / elapsedSeconds; // 计算FPS
            std::cout << "原始相机取流FPS: " << fps1 << std::endl;

            lastTime1 = now1; // 重置计时器
            frameCount1 = 0;} // 重置帧数计数器
*/

        // 使用锁确保线程安全地向队列添加图像
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            // 清空队列，只保留最新的一帧图像
            std::queue<cv::Mat> empty;
            std::swap(frame_queue, empty);
            frame_queue.push(resizedImage.clone());
        }
    }
}

// 图像处理函数
void ProcessImage(cv::Mat& img, InferRequestPool& inferRequestPool) {
    if (img.empty()) return; // 确保图像非空

    latestImage = img.clone(); // 存储最新的图像副本供以后使用
    // 提交图像帧进行推理
    inferRequestPool.submitInferRequest(img);

    // 显示处理后的图像（如果有物体被检测到）
    /*
    cv::imshow("Detection", img);
    cv::waitKey(1);
    */
}

void InferRequestPool::submitInferRequest(const cv::Mat& img) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !idleRequests_.empty(); });

    size_t index = idleRequests_.front();
    idleRequests_.pop();

    auto& inferRequest = inferRequests_[index];
    // 图像预处理逻辑
    cv::Mat blob = cv::dnn::blobFromImage(img, 1.0 / 255.0, cv::Size(img_width, img_height), cv::Scalar(), true, false);
    inferRequest.set_input_tensor(ov::Tensor(compiledModel.input().get_element_type(), {1, 3, img_height, img_width}, blob.data));
    inferRequest.start_async();
}

void InferRequestPool::processResult(size_t index) {
    auto& inferRequest = inferRequests_[index];
    auto output = inferRequest.get_output_tensor();
    auto output_shape = output.get_shape();

    float* data = output.data<float>();
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
    cv::transpose(output_buffer, output_buffer);
    float score_threshold = 0.6;
    std::vector<int> class_ids;
    std::vector<float> class_scores;
    std::vector<cv::Rect> boxes;
    gary_msgs::msg::AutoAIM msg;

    // Bypass this to ros speak out
    cv::Point class_id_point_outside;
    double maxClassScore_outside;
    int width_outside;

    for (int i = 0; i < output_buffer.rows; i++) {
        cv::Mat classes_scores = output_buffer.row(i).colRange(4, output_buffer.cols);
        cv::Point class_id_point;
        double maxClassScore;
        cv::minMaxLoc(classes_scores, nullptr, &maxClassScore, nullptr, &class_id_point);
        if (maxClassScore > score_threshold) {
            float cx = output_buffer.at<float>(i, 0);
            float cy = output_buffer.at<float>(i, 1);
            float w = output_buffer.at<float>(i, 2);
            float h = output_buffer.at<float>(i, 3);
            int left = int(cx - 0.5 * w);
            int top = int(cy - 0.5 * h);
            int width = int(w);
            int height = int(h);

            class_ids.push_back(class_id_point.x);
            class_scores.push_back(maxClassScore);
            boxes.push_back(cv::Rect(left, top, width, height));

            //update this to global vals
            class_id_point_outside = class_id_point;
            maxClassScore_outside = maxClassScore;
            width_outside = width;
            /*
            std::cout << "Detected class " << class_id_point.x << " with confidence " << maxClassScore 
                      << " at [" << left << ", " << top << ", " << width << ", " << height << "]" << std::endl;
            */
        }
    }

    // 线程安全地复制最新图像
    cv::Mat safeCopy;
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        safeCopy = latestImage.clone();
    }

    for (size_t i = 0; i < class_ids.size(); i++) {
        if (class_ids[i] == 0 || class_ids[i] == 1) {
            cv::Mat targetArea = safeCopy(boxes[i]);

            // 转换到HSV色彩空间进行颜色检测
            cv::Mat targetAreaHSV;
            cv::cvtColor(targetArea, targetAreaHSV, cv::COLOR_BGR2HSV);

            cv::Mat mask;
            if (class_ids[i] == 1) {
                // 红色灯条的HSV范围
                cv::inRange(targetAreaHSV, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask);
            } else if (class_ids[i] == 0) {
                // 蓝色灯条的HSV范围
                cv::inRange(targetAreaHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), mask);
            }

            cv::Mat dilated, eroded, reDilated;
            cv::dilate(mask, dilated, cv::Mat(), cv::Point(-1, -1), 2);
            // 执行腐蚀操作
            cv::erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1), 2); // 迭代2次进行腐蚀
            // 在腐蚀之后增加膨胀的迭代次数来填充内部空白
            cv::dilate(eroded, reDilated, cv::Mat(), cv::Point(-1, -1), 3); // 迭代3次进行再膨胀

            // 更新processedImage以显示腐蚀膨胀后的结果（测试用途）
            {
                std::lock_guard<std::mutex> lock(imageMutex2);
                processedImage2 = eroded.clone(); // 注意，这会覆盖原有的processedImage内容
            }

            // 轮廓检测
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(eroded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 根据轮廓面积过滤并进行初步处理
            std::vector<cv::Rect> preliminaryFilteredBoxes;
            for (auto& contour : contours) {
                cv::Rect box = cv::boundingRect(contour);
                if (box.height > 20 && box.width > 10 && box.width < box.height) { // 灯条高度至少为20且高度大于宽度
                    preliminaryFilteredBoxes.push_back(box);
                }
            }

            // 合并距离过近的轮廓
            std::vector<cv::Rect> mergedBoxes;
            for (auto& box : preliminaryFilteredBoxes) {
                bool merged = false;
                for (auto& mergedBox : mergedBoxes) {
                    // 检查最小距离条件（两个灯条的最小距离大于等于它们的高度）
                    int distance = std::abs((mergedBox.x + mergedBox.width / 2) - (box.x + box.width / 2));
                    if (distance < std::max(mergedBox.height, box.height)) {
                        // 合并逻辑：扩展mergedBox以包含当前box
                        mergedBox = mergedBox | box; // 使用cv::Rect的|运算符来合并两个矩形
                        merged = true;
                        break;
                    }
                }
                if (!merged) {
                    mergedBoxes.push_back(box);
                }
            }

            // 确保检测到两条灯条
            if (mergedBoxes.size() == 2) {
                // 按x坐标排序以确定左右灯条
                std::sort(mergedBoxes.begin(), mergedBoxes.end(), [](const cv::Rect& a, const cv::Rect& b) {
                    return a.x < b.x;
                });

                // 取最左边和最右边的两个灯条
                cv::Rect leftBar = mergedBoxes.front();
                cv::Rect rightBar = mergedBoxes.back();

                // 计算中心点
                cv::Point center(
                    (leftBar.x + leftBar.width / 2 + boxes[i].x + rightBar.x + rightBar.width / 2 + boxes[i].x) / 2,
                    (leftBar.y + leftBar.height / 2 + boxes[i].y + rightBar.y + rightBar.height / 2 + boxes[i].y) / 2

                );

                // 在safeCopy上绘制中心点
                cv::circle(safeCopy, center, 5, cv::Scalar(0, 0, 255), -1);

                // 打印灯条的坐标
                //std::cout << "Left bar top:" << leftBar.tl() << ", Left bar bottom:" << leftBar.br() << std::endl;
                cv::Point left_top = leftBar.tl();
                cv::Point left_bottom = leftBar.br();
                std::string left_Bra_top_str = "(" + std::to_string(left_top.x) + "," + std::to_string(left_top.y) + ")";
                std::string left_Bra_bottom_str = "(" + std::to_string(left_bottom.x) + "," + std::to_string(left_bottom.y) + ")";

                //std::cout << "Right bar top:" << rightBar.tl() << ", Right bar bottom:" << rightBar.br() << std::endl;
                cv::Point right_top = rightBar.tl();
                cv::Point right_bottom = rightBar.br();
                std::string right_Bra_top_str = "(" + std::to_string(right_top.x) + "," + std::to_string(right_top.y) + ")";
                std::string right_Bra_bottom_str = "(" + std::to_string(right_bottom.x) + "," + std::to_string(right_bottom.y) + ")";

                // 如果需要，可以在此处保存四个顶点坐标以用于后续的复杂预测
                // 注意，这里已经是根据排序后的灯条来确定左右，因此直接使用leftBar和rightBar的坐标即可
                // 计算顶点时需要加上检测框的左上角坐标来转换为绝对坐标
                // 左灯条顶部点
                cv::Point topLeft(leftBar.x + boxes[i].x, leftBar.y + boxes[i].y);
                // 右灯条顶部点
                cv::Point topRight(rightBar.x + rightBar.width + boxes[i].x, rightBar.y + boxes[i].y);
                // 左灯条底部点
                cv::Point bottomLeft(leftBar.x + boxes[i].x, leftBar.y + leftBar.height + boxes[i].y);
                // 右灯条底部点
                cv::Point bottomRight(rightBar.x + rightBar.width + boxes[i].x, rightBar.y + rightBar.height + boxes[i].y);

                //预测框中点
                cv::Point center_ori = (boxes[i].tl() + boxes[i].br()) / 2;

                // 将中心点坐标转换为字符串表示形式
                std::string centerOriStr = "(" + std::to_string(center_ori.x) + ", " + std::to_string(center_ori.y) + ")";

                // 可以选择绘制五个角点，以可视化检查
                cv::circle(safeCopy, topLeft, 3, cv::Scalar(255, 255, 255), -1); // 左上角
                cv::circle(safeCopy, topRight, 3, cv::Scalar(255, 255, 255), -1); // 右上角
                cv::circle(safeCopy, bottomLeft, 3, cv::Scalar(255, 255, 255), -1); // 左下角
                cv::circle(safeCopy, bottomRight, 3, cv::Scalar(255, 255, 255), -1); // 右下角
                cv::circle(safeCopy, center_ori, 5, cv::Scalar(0, 255, 255), -1); //推算中点

                // 相机内参和畸变系数
                cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1419.3, 0, 611.3, 0, 1418.2, 533.1, 0, 0, 1);
                cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.1295, 0.0804, 4.85E-04, 6.37E-04, 0.2375);

                // 物体实际宽度（米）
                float objectWidth = 0.079; // 0.125m

                // 计算距离
                float focalLength = (cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(1,1)) / 2; // 焦距平均值
                float distance = (objectWidth * focalLength) / width_outside;

                // 计算yaw和pitch（简化计算，实际情况可能需要更精确的方法）
                float yaw = 0.9 * atan((2*center_ori.x - 640) / focalLength);
                float pitch = 0.9 * atan((2*center_ori.y - 512) / focalLength);

                yaw = yaw + yaw_act;
                pitch = pitch + pitch_act;

                // 输出中心点坐标
                //std::cout << "Center calculated:" << centerOriStr << std::endl;
                // 将中心点坐标添加到消息中
                //msg.data = "Type:" + std::to_string(class_id_point_outside.x) + ",Center calculated:" + centerOriStr + ",Confidence:" + std::to_string(maxClassScore_outside) + ",Left bar top:" + left_Bra_top_str + ",Left bar bottom:" + left_Bra_bottom_str + ",Right bar top:" + right_Bra_top_str + ",Right bar bottom:" + right_Bra_bottom_str + ",Distance:" + std::to_string(distance) + ",Yaw:" + std::to_string(yaw) + ",Pitch:" + std::to_string(pitch);
                //推送消息
                msg.yaw = yaw;
                msg.pitch = pitch;
                msg.target_distance = distance;
                publisher_->publish(msg);

            }
            else {
                cv::Point center = (boxes[i].tl() + boxes[i].br()) / 2;
                cv::circle(safeCopy, center, 5, cv::Scalar(0, 215, 255), -1);
                std::string centerStr = "(" + std::to_string(center.x) + ", " + std::to_string(center.y) + ")";


                // 相机内参和畸变系数
                cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1419.3, 0, 611.3, 0, 1418.2, 533.1, 0, 0, 1);
                cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.1295, 0.0804, 4.85E-04, 6.37E-04, 0.2375);

                // 物体实际宽度（米）
                float objectWidth = 0.079; // 0.125m

                // 计算距离
                float focalLength = (cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(1,1)) / 2; // 焦距平均值
                float distance = (objectWidth * focalLength) / width_outside;

                // 计算yaw和pitch（简化计算，实际情况可能需要更精确的方法）
                float yaw = 0.7 * atan((2*center.x - 640) / focalLength);
                float pitch = 0.7 *atan((2*center.y - 512) / focalLength);

                yaw = yaw + yaw_act;
                pitch = pitch + pitch_act;

                //std::cout << "Distance: " << distance << "m\n";
                //'std::cout << "Yaw: " << yaw << " degrees\n";
                //std::cout << "Pitch: " << pitch << " degrees\n";

                //std::cout << "Center original:" << centerStr << std::endl;
                //msg.data = "Type:" + std::to_string(class_id_point_outside.x) + ",Center original:" + centerStr + ",Confidence:" + std::to_string(maxClassScore_outside) + ",Distance:" + std::to_string(distance) + ",Yaw:" + std::to_string(yaw) + ",Pitch:" + std::to_string(pitch);
                msg.yaw = yaw;
                msg.pitch = pitch;
                msg.target_distance = distance;

                publisher_->publish(msg);
            }
        }
        
        // 更新全局的processedImage变量
    
        {
        std::lock_guard<std::mutex> lock(imageMutex);
        processedImage = safeCopy.clone();
        }
    
    }

}

// 相机捕获线程
void CaptureThread(Camera& camera, ov::CompiledModel& compiled_model, InferRequestPool& inferRequestPool) {
    while (g_bAcquisitionFlag) {
        cv::Mat frame;
        // 从相机或队列中获取图像帧
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!frame_queue.empty()) {
                frame = frame_queue.front();
                frame_queue.pop();
            }
        }
        if (!frame.empty()) {
            ProcessImage(frame, inferRequestPool);
        }
    }
}

InferRequestPool::InferRequestPool(ov::CompiledModel& compiledModel, size_t poolSize, rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr publisher)
    : compiledModel(compiledModel), img_width(640), img_height(512),
      publisher_(publisher) // 初始化列表应放在冒号后面
{
    for (size_t i = 0; i < poolSize; ++i) {
        auto inferRequest = compiledModel.create_infer_request();
        inferRequest.set_callback([this, i](std::exception_ptr ptr) {
            if (!ptr) {
                try {
                    processResult(i);
                } catch (...) {
                    // 处理 processResult 中的异常
                }
            } else {
                try {
                    std::rethrow_exception(ptr);
                } catch (const std::exception& e) {
                    // 处理推理请求中的异常
                    std::cerr << "推理请求异常: " << e.what() << std::endl;
                }
            }
            // 将请求标记为闲置
            {
                std::lock_guard<std::mutex> lock(mutex_);
                idleRequests_.push(i);
            }
            condition_.notify_one();
        });
        inferRequests_.push_back(std::move(inferRequest));
        idleRequests_.push(i);
    }
}



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("vision_quner_node");
    auto publisher = node->create_publisher<gary_msgs::msg::AutoAIM>("autoaim/target", 1);
    auto subscription_yaw = node->create_subscription<gary_msgs::msg::DualLoopPIDWithFilter>(
        "gimbal_yaw_pid/pid", 1, callback_yaw);
    auto subscription_pitch = node->create_subscription<gary_msgs::msg::DualLoopPIDWithFilter>(
        "gimbal_pitch_pid/pid", 1, callback_pitch);

    system("clear");
    std::cout << "\n";
    std::cout << "---------------------------------------------------------------------------------\n";
    std::cout << "---------------------------------------------------------------------------------\n";
    std::cout << "\n";
    std::cout << "qqqqq   u   u  n   n   eeeee   rrrr      V     V  iii   ssss   iii   ooo   n   n \n";
    std::cout << "q   q   u   u  nn  n   e       r   r      V   V    i   s        i   o   o  nn  n \n";
    std::cout << "q   q   u   u  n n n   eeeee   rrrr        V V     i    ssss    i   o   o  n n n \n";
    std::cout << "q  qq   u   u  n  nn   e       r r          V      i        s   i   o   o  n  nn \n";
    std::cout << "qqqq q   uuu   n   n   eeeee   r  rr        V     iii   ssss   iii   ooo   n   n \n";
    std::cout << "\n";
    std::cout << "\n";
    std::cout << "q u n e r C l o u d . c o m\n";
    std::cout << "---------------------------------------------------------------------------------\n";
    std::cout << "---------------------------------------------------------------------------------\n";

    // 创建Camera实例并尝试打开相机
    Camera camera;
    if (!camera.open()) {
        std::cerr << "Failed to open camera.\n";
        return -1;
    }
    camera.startCapture();
    std::cout << "Camera opened successfully.\n";
    // 初始化OpenVINO Runtime Core并加载模型
    
    ov::Core core;
    auto model = core.read_model("/home/gmaster/Robomaster_2024_XJTLU_Vision/final_model/quner_INT8.xml");
    auto compiled_model = core.compile_model(model, "MULTI:GPU,CPU");
    std::cout << "Model compiled successfully.\n";

    InferRequestPool inferRequestPool(compiled_model, quner_multi, publisher);
    
    std::thread capture_thread([&]() { 
    CaptureThread(camera, compiled_model, inferRequestPool);
    });
    
    std::thread subscription_thread([&]() {
        rclcpp::spin(node); // 在单独的线程中执行 ROS 节点的事件循环
    });

    std::cout << "Multi Thread Boosted.Press Enter to stop...\n";
    std::cin.get();

        // 创建一个循环来显示图像，直到收到停止信号
    /*
    while (g_bAcquisitionFlag) {
        //原图像
        cv::Mat imageToShow;
        {
            std::lock_guard<std::mutex> lock(imageMutex);
            if (!processedImage.empty()) {
                imageToShow = processedImage.clone();
            }
        }
        //腐蚀后
        cv::Mat imageToShow2;
        {
            std::lock_guard<std::mutex> lock(imageMutex2);
            if (!processedImage2.empty()) {
                imageToShow2 = processedImage2.clone();
            }
        }

        if (!imageToShow.empty() && !imageToShow2.empty()) {
        
            //cv::imshow("Detection Results", imageToShow);
            //cv::imshow("morphOpsImageToShow", imageToShow2);
            //cv::waitKey(1);
        
        }
    }
    */
    g_bAcquisitionFlag = false;
    capture_thread.join();
    subscription_thread.join();
    rclcpp::shutdown();
    std::cout << "ROS closed.\n";
    camera.close();
    std::cout << "Camera capture stopped and camera closed.\n";
    return 0;
}



//                            _ooOoo_  
//                           o8888888o  
//                           88" . "88  
//                           (| -_- |)  
//                            O\ = /O  
//                        ____/`---'\____  
//                      .   ' \\| |// `.  
//                       / \\||| : |||// \  
//                     / _||||| -:- |||||- \  
//                       | | \\\ - /// | |  
//                     | \_| ''\---/'' | |  
//                      \ .-\__ `-` ___/-. /  
//                   ___`. .' /--.--\ `. . __  
//                ."" '< `.___\_<|>_/___.' >'"".  
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |  
//                 \ \ `-. \_ __\ /__ _/ .-` / /  
//         ======`-.____`-.___\_____/___.-`____.-'======  
//                            `=---='  
//  
//         .............................................  
//                  佛祖镇楼                 BUG辟易  
//                         以下是函数，请勿乱改

//人生若只如初见，
//何事秋风悲画扇。
//等闲变却故人心，
//却道故人心易变。
//骊山语罢清宵半，
//泪雨霖铃终不怨。
//何如薄幸锦衣郎，
//比翼连枝当日愿。
