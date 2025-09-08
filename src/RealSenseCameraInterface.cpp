/** @file RealSenseCameraInterface.cpp
 *  @brief RealSense相机接口实现
 *  @date 2025.09.05
 */

#include "RealSenseCameraInterface.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include "pc_LUT.h"

/** @brief 构造函数，初始化相机接口 */
RealSenseCameraInterface::RealSenseCameraInterface()
{
    pipe_ = std::make_unique<rs2::pipeline>(ctx_);
}

/** @brief 析构函数，释放相机资源 */
RealSenseCameraInterface::~RealSenseCameraInterface()
{
    disconnect();
}

/** @brief 扫描可用的RealSense相机
 *  @return 返回发现的相机信息列表
 */
std::vector<CameraInfo> RealSenseCameraInterface::scanCameras()
{
    std::vector<CameraInfo> cameras;
    auto devices = ctx_.query_devices();
    for (size_t i = 0; i < devices.size(); ++i)
    {
        CameraInfo info;
        info.serial_number = devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        info.vendor = devices[i].get_info(RS2_CAMERA_INFO_NAME);
        info.manufacturer = devices[i].get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        cameras.push_back(info);
        std::cout << "    ✔ [" << i << "] Camera found: Serial Number=" << info.serial_number << ", Vendor=" << info.vendor
                  << ", Firmware Version=" << info.manufacturer << std::endl;
    }
    return cameras;
}

/** @brief 选择指定序列号的相机
 *  @param sn 相机序列号
 *  @return 是否选择成功
 */
bool RealSenseCameraInterface::selectCamera(const std::string& sn)
{
    selected_sn_ = sn;
    auto devices = ctx_.query_devices();
    for (const auto& dev : devices)
    {
        if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == sn)
        {
            selected_name_ = dev.get_info(RS2_CAMERA_INFO_NAME);
            std::cout << "    ✔ Camera selected: Serial Number=" << selected_sn_ << ", Model=" << selected_name_ << std::endl;            return true;
        }
    }
    std::cerr << "    ✘ Camera not found: Serial Number=" << sn << std::endl;
    return false;
}

/** @brief 执行相机硬件复位
 *  @return 是否复位成功
 */
bool RealSenseCameraInterface::hardwareReset()
{
    auto devices = ctx_.query_devices();
    for (size_t i = 0; i < devices.size(); ++i)
    {
        auto dev = devices[i];
        if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == selected_sn_)
        {
            try
            {
                dev.hardware_reset();
                std::cout << "🔧️ Performing hardware reset..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
                return true;
            }
            catch (const rs2::error& e)
            {
                std::cerr << "✘ Hardware reset failed: " << e.what() << std::endl;
                return false;
            }
        }
    }
    std::cerr << "✘ Camera to reset not found" << std::endl;
    return false;
}

/** @brief 连接到选定的相机
 *  @return 是否连接成功
 */
bool RealSenseCameraInterface::connect()
{
    if (selected_sn_.empty())
    {
        std::cerr << "    ✘ Error: No camera selected [0x03210008]" << std::endl;
        return false;
    }
    if (status_ == CAMERA_STATUS_CONNECTED)
    {
        std::cerr << "    ✘ Camera initialization failed: RealSense camera already connected [0x03210008]" << std::endl;
        return false;
    }

    std::cout << "    ✔ Starting connection..." << std::endl;
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    auto devices = ctx_.query_devices();
    int devices_cnt = devices.size();

    std::cout << "    ✔ Found " << devices_cnt << " device(s)" << std::endl;
    if (devices_cnt <= 0)
    {
        std::cerr << "    ✘ Camera initialization failed: No RealSense camera found [0x03210009]" << std::endl;        return false;
    }

    int id = -1;
    for (int i = 0; i < devices_cnt; ++i)
    {
        rs2::device cur_dev = devices[i];
        std::string serial = cur_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (serial == selected_sn_)
        {
            id = i;
            selected_name_ = cur_dev.get_info(RS2_CAMERA_INFO_NAME);
            break;
        }
    }
    if (id == -1)
    {
        std::cerr << "    ✘ Camera not found: Serial Number=" << selected_sn_ << " [0x03210009]" << std::endl;        return false;
    }

    device_ = devices[id];
    sensors_ = device_.query_sensors();
    std::cout << "    ✔ Will connect to Serial Number=" << selected_sn_ << ", Model=" << selected_name_ << std::endl;
    // 配置流
    cfg_.enable_device(selected_sn_);

    try
    {
        if (params_.enable_pointcloud)
        {
            if (selected_name_ == "Intel RealSense L515")
            {
                cfg_.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
            }
            else if (selected_name_ == "Intel RealSense D435" || selected_name_ == "Intel RealSense D435I")
            {
                cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            }
        }
        if (params_.enable_ir_left)
        {
            if (selected_name_ == "Intel RealSense L515")
            {
                cfg_.enable_stream(RS2_STREAM_INFRARED, 1024, 768, RS2_FORMAT_Y8, 30);
            }
            else if (selected_name_ == "Intel RealSense D435" || selected_name_ == "Intel RealSense D435I")
            {
                cfg_.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
            }
        }
        if (params_.enable_rgb)
        {
            cfg_.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
        }

        pipe_->start(cfg_);
    }
    catch (const rs2::error& e)
    {
        std::cerr << "    ✘ Failed to start pipeline: " << e.what() << std::endl;
        // 停止旧管道，避免 USB 忙
        try
        {
            pipe_->stop();
        }
        catch (...) {}

        // 尝试硬件复位
        if (hardwareReset())
        {
            std::cout << "    🔧️ Retrying connection after reset..." << std::endl;
            pipe_.reset(new rs2::pipeline(ctx_));
            return connect(); // 递归重试
        }
        else
        {
            return false;
        }
    }

    status_ = CAMERA_STATUS_CONNECTED;
    callback_disconnected_ = false;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 注册设备变化回调
    ctx_.set_devices_changed_callback([this](rs2::event_information info) mutable
    {
        if (info.was_removed(device_))
        {
            callback_disconnected_ = true;
            std::cout << "---Camera Disconnection Callback---" << std::endl;
            status_ = CAMERA_STATUS_LOST_CONNECTION;
        }
    });

    return true;
}

/** @brief 配置相机参数
 *  @param params 相机参数结构体
 *  @return 是否配置成功
 */
bool RealSenseCameraInterface::configureParameters(const CameraParam& params)
{
    if (status_ != CAMERA_STATUS_CONNECTED)
    {
        std::cerr << "    ✘ Failed to configure parameters: Camera not connected" << std::endl;
        return false;
    }

    // 停止管道，避免 USB 忙
    try
    {
        pipe_->stop();
        std::cout << "    ✔ Temporarily stopping pipeline for parameter configuration" << std::endl;
    }
    catch (const rs2::error& e)
    {
        std::cerr << "    ✘ Failed to stop pipeline: " << e.what() << std::endl;
        return false;
    }

    // 应用参数到传感器
    params_ = params;

    // 根据相机型号设置参数
    if (selected_name_ == "Intel RealSense L515")
    {
        // L515深度传感器参数设置
        sensors_[0].set_option(RS2_OPTION_MIN_DISTANCE, params_.min_distance);
        sensors_[0].set_option(RS2_OPTION_POST_PROCESSING_SHARPENING, params_.post_processing_sharpening);
        sensors_[0].set_option(RS2_OPTION_PRE_PROCESSING_SHARPENING, params_.pre_processing_sharpening);
        sensors_[0].set_option(RS2_OPTION_NOISE_FILTERING, params_.noise_filtering);
        sensors_[0].set_option(RS2_OPTION_CONFIDENCE_THRESHOLD, params_.confidence_threshold);
        sensors_[0].set_option(RS2_OPTION_LASER_POWER, params_.laser_power);

        // L515彩色传感器参数设置
        if (sensors_[1].supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            sensors_[1].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, params_.auto_exposure);
        }
        if (!params_.auto_exposure)
        {
            sensors_[1].set_option(RS2_OPTION_EXPOSURE, params_.exposure);
            sensors_[1].set_option(RS2_OPTION_GAIN, params_.gain);
        }
        sensors_[1].set_option(RS2_OPTION_BRIGHTNESS, params_.brightness);
        sensors_[1].set_option(RS2_OPTION_CONTRAST, params_.contrast);
        sensors_[1].set_option(RS2_OPTION_SHARPNESS, params_.sharpness);
    }
    else if (selected_name_ == "Intel RealSense D435" || selected_name_ == "Intel RealSense D435I")
    {
        // D435/D435I深度传感器参数设置
        if (sensors_[0].supports(RS2_OPTION_LASER_POWER))
        {
            sensors_[0].set_option(RS2_OPTION_LASER_POWER, params_.laser_power);
        }
        if (sensors_[0].supports(RS2_OPTION_MIN_DISTANCE))
        {
            sensors_[0].set_option(RS2_OPTION_MIN_DISTANCE, params_.min_distance);
        }
        if (sensors_[0].supports(RS2_OPTION_POST_PROCESSING_SHARPENING))
        {
            sensors_[0].set_option(RS2_OPTION_POST_PROCESSING_SHARPENING, params_.post_processing_sharpening);
        }
        if (sensors_[0].supports(RS2_OPTION_PRE_PROCESSING_SHARPENING))
        {
            sensors_[0].set_option(RS2_OPTION_PRE_PROCESSING_SHARPENING, params_.pre_processing_sharpening);
        }
        if (sensors_[0].supports(RS2_OPTION_NOISE_FILTERING))
        {
            sensors_[0].set_option(RS2_OPTION_NOISE_FILTERING, params_.noise_filtering);
        }
        if (sensors_[0].supports(RS2_OPTION_CONFIDENCE_THRESHOLD))
        {
            sensors_[0].set_option(RS2_OPTION_CONFIDENCE_THRESHOLD, params_.confidence_threshold);
        }

        // D435/D435I彩色传感器参数设置
        if (sensors_[1].supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            sensors_[1].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, params_.auto_exposure ? 1.0f : 0.0f);
        }
        if (!params_.auto_exposure)
        {
            if (sensors_[1].supports(RS2_OPTION_EXPOSURE))
            {
                sensors_[1].set_option(RS2_OPTION_EXPOSURE, params_.exposure);
            }
            if (sensors_[1].supports(RS2_OPTION_GAIN))
            {
                sensors_[1].set_option(RS2_OPTION_GAIN, params_.gain);
            }
        }
        if (sensors_[1].supports(RS2_OPTION_BRIGHTNESS))
        {
            sensors_[1].set_option(RS2_OPTION_BRIGHTNESS, params_.brightness);
        }
        if (sensors_[1].supports(RS2_OPTION_CONTRAST))
        {
            sensors_[1].set_option(RS2_OPTION_CONTRAST, params_.contrast);
        }
        if (sensors_[1].supports(RS2_OPTION_SHARPNESS))
        {
            sensors_[1].set_option(RS2_OPTION_SHARPNESS, params_.sharpness);
        }
    }
    else
    {
        std::cerr << "    ✘ Unsupported camera model: " << selected_name_ << std::endl;
        return false;
    }

    // 重新启动管道
    pipe_.reset(new rs2::pipeline(ctx_));
    try
    {
        pipe_->start(cfg_);
        std::cout << "    ✔ Restarting pipeline after parameter configuration" << std::endl;
    }
    catch (const rs2::error& e)
    {
        std::cerr << "    ✘ Failed to restart pipeline: " << e.what() << std::endl;
        return false;
    }

    std::cout << "    ✔ Parameters configured successfully" << std::endl;
    return true;
}

/** @brief 捕获一帧数据（RGB、红外、点云）
 *  @param outputs 存储捕获的输出数据
 *  @return 是否捕获成功
 */
bool RealSenseCameraInterface::capture(std::map<std::string, std::shared_ptr<void>>& outputs)
{
    if (status_ != CAMERA_STATUS_CONNECTED)
    {
        std::cerr << "    ✘ Capture failed: Camera not connected" << std::endl;
        status_ = CAMERA_STATUS_LOST_CONNECTION;
        return false;
    }
    if (is_capturing_)
    {
        std::cerr << "    ✘ Capture failed: Previous capture not completed" << std::endl;
        return false;
    }

    is_capturing_ = true;
    rs2::align align(params_.align_method == 0 ? RS2_STREAM_INFRARED : RS2_STREAM_COLOR);
    rs2::pointcloud pc;

    try
    {
        rs2::frameset frames = pipe_->wait_for_frames();

        rs2::video_frame color = frames.get_color_frame();
        rs2::video_frame ir_left = frames.get_infrared_frame();
        rs2::frameset aligned_frames = align.process(frames);
        rs2::depth_frame depth = aligned_frames.get_depth_frame();

        if (params_.enable_rgb && color)
        {
            int width = color.get_width();
            int height = color.get_height();
            int bytes_per_pixel = color.get_bytes_per_pixel();
            char* data = (char*)malloc(width * height * bytes_per_pixel);
            memcpy(data, color.get_data(), width * height * bytes_per_pixel);

            cv::Mat rgb = cv::Mat(height, width, CV_8UC3, data);
            cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
            outputs["rgb"] = std::make_shared<cv::Mat>(rgb);// ******> 输出彩色图像 <******
            std::cout << "    ✔ Captured RGB image successfully" << std::endl;
        }
        else if (params_.enable_rgb)
        {
            std::cerr << "    ✘ Failed to capture RGB image" << std::endl;
        }

        if (params_.enable_ir_left && ir_left)
        {
            int width = ir_left.get_width();
            int height = ir_left.get_height();
            int bytes_per_pixel = ir_left.get_bytes_per_pixel();
            char* data = (char*)malloc(width * height * bytes_per_pixel);
            memcpy(data, ir_left.get_data(), width * height * bytes_per_pixel);

            cv::Mat gray = cv::Mat(height, width, CV_8UC1, data);
            outputs["ir_left"] = std::make_shared<cv::Mat>(gray);// ******> 输出左红外图像 <******
            std::cout << "    ✔ Captured infrared image successfully" << std::endl;
        }
        else
        {
            std::cerr << "    ✘ Failed to capture infrared image" << std::endl;
        }

        if (params_.enable_pointcloud && depth)
        {
            // 点云查找表
            std::shared_ptr<PointCloudRegLUT> pointcloudLUT = std::make_shared<PointCloudRegLUT>();
            int width = depth.get_width();
            int height = depth.get_height();

            if (color)
            {
                pc.map_to(color);// 将点云映射到彩色图像
            }
            else if (ir_left)
            {
                pc.map_to(ir_left);
            }
            auto points = pc.calculate(depth);
            auto vertices = points.get_vertices();
            // auto pc_out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // 纯点云类型
            auto pc_out = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(); // 支持RGB的点云类型
            pc_out->reserve(points.size());

            int num=-1;
            for (size_t i = 0; i < points.size(); ++i)
            {
                bool none_empty=false;
                float x = 1000 * vertices[i].x;
                float y = 1000 * vertices[i].y;
                float z = 1000 * vertices[i].z;

                pcl::PointXYZRGB point; // RGB 点

                if(params_.enable_pointcloudLUT)
                {
                    if(fabs(x) > 1e-6 && fabs(y) > 1e-6 && fabs(z) > 1e-6  && fabs(z) < 1000) {
                        num++;
                        point.x = x;
                        point.y = y;
                        point.z = z;
                        none_empty=true;
                    }

                    int w_index = i%width;
                    int h_index = i/width;

                    // 从彩色图像获取像素颜色
                    if (none_empty && color)
                    {
                        const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
                        int pixel_index = (h_index * width + w_index) * 3; // 每个像素有3个通道 (RGB)
                        uint8_t r = color_data[pixel_index];     // 红色通道
                        uint8_t g = color_data[pixel_index + 1]; // 绿色通道
                        uint8_t b = color_data[pixel_index + 2]; // 蓝色通道

                        point.r = r;
                        point.g = g;
                        point.b = b;
                    }

                    // 如果点有效，则添加到点云
                    if (none_empty)
                    {
                        pc_out->push_back(point);
                    }

                    // 保证 vector 有足够的行
                    if (h_index >= pointcloudLUT->size()) {
                        pointcloudLUT->resize(h_index + 1);
                    }

                    // 保证 vector 有足够的列
                    if (w_index >= (*pointcloudLUT)[h_index].size()) {
                        (*pointcloudLUT)[h_index].resize(w_index + 1);
                    }

                    // 赋值
                    if(none_empty)
                        (*pointcloudLUT)[h_index][w_index] = num;
                    else
                        (*pointcloudLUT)[h_index][w_index] = -1;
                    outputs["pointcloud_lut"] = pointcloudLUT;// ******> 输出点云查找表 <******
                }
                else
                {
                    if(fabs(x) > 1e-6 && fabs(y) > 1e-6 && fabs(z) > 1e-6 && fabs(z) < 1000)
                    {
                        point.x = x;
                        point.y = y;
                        point.z = z;

                        // 如果有彩色图像，取相应的颜色
                        if (color)
                        {
                            // 获取彩色图像的像素颜色
                            const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
                            int pixel_index = i * 3; // 每个像素有3个通道 (RGB)
                            uint8_t r = color_data[pixel_index];     // 红色通道
                            uint8_t g = color_data[pixel_index + 1]; // 绿色通道
                            uint8_t b = color_data[pixel_index + 2]; // 蓝色通道

                            point.r = r;
                            point.g = g;
                            point.b = b;
                        }

                        pc_out->push_back(point);
                    }
                }
            }
            if (pc_out->empty())
            {
                std::cerr << "    ✘ Point cloud is empty after filtering" << std::endl;
            }
            else
            {
                outputs["pointcloud"] = pc_out;// ******> 输出点云 <******
                std::cout << "    ✔ Captured point cloud, containing " << pc_out->size() << " points" << std::endl;
            }
        }
        else if (params_.enable_pointcloud)
        {
            std::cerr << "Failed to capture depth frame, cannot generate point cloud" << std::endl;
        }

        status_ = CAMERA_STATUS_CAPTURE_SUCCESS;
        is_capturing_ = false;
        return true;
    }
    catch (const rs2::error& e)
    {
        status_ = CAMERA_STATUS_CAPTURE_FAILED;
        is_capturing_ = false;
        return false;
    }
    return true;
}

/** @brief 断开相机连接
 *  @return 是否断开成功
 */
bool RealSenseCameraInterface::disconnect()
{
    if (status_ == CAMERA_STATUS_CONNECTED)
    {
        try
        {
            pipe_->stop();
            status_ = CAMERA_STATUS_INIT;
            sensors_.clear();
            std::cout << "    ✔ Camera disconnected" << std::endl;
            return true;
        }
        catch (const rs2::error& e)
        {
            std::cerr << "    ✘ Disconnection failed: " << e.what() << std::endl;
            return false;
        }
    }
    return true;
}

/** @brief 获取相机当前状态
 *  @return 相机状态
 */
CameraStatus RealSenseCameraInterface::getStatus()
{
    return status_;
}

std::string RealSenseCameraInterface::getTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time = *std::localtime(&time);

    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch())%1000;

    std::ostringstream oss;
    oss << std::put_time(&local_time,"%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3) << milliseconds.count();
    return oss.str();
}