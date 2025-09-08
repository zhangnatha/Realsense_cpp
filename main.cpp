/** @file main.cpp
 *  @brief RealSense相机接口示例：自动保存一帧RGB、红外和点云
 *  @date 2025.09.07
 */

#include "RealSenseCameraInterface.h"
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <sstream>
#include <thread>

#include "pc_LUT.h"

int main()
{
    RealSenseCameraInterface camera;

    // 步骤1: 扫描相机
    std::cout << "🚀 Step 1: Scanning cameras ..." << std::endl;
    auto cameras = camera.scanCameras();
    if (cameras.empty())
    {
        std::cerr << "No cameras found" << std::endl;
        return -1;
    }

    // 步骤2: 选择相机
    int camera_index;
    std::cout << "🚀 Step 2:Enter camera index (0 to " << cameras.size() - 1 << "): ";
    std::cin >> camera_index;
    if (camera_index < 0 || camera_index >= static_cast<int>(cameras.size()))
    {
        std::cerr << "    ✘ Invalid camera index" << std::endl;
        return -1;
    }
    if (!camera.selectCamera(cameras[camera_index].serial_number))
    {
        std::cerr << "    ✘ Failed to select camera" << std::endl;
        return -1;
    }

    // 步骤3: 连接相机
    std::cout << "🚀 Step 3: Connecting camera ..." << std::endl;
    if (!camera.connect())
    {
        std::cerr << "    ✘ Failed to connect camera" << std::endl;
        return -1;
    }

    // 步骤4: 配置参数
    std::cout << "🚀 Step 4: Configuring parameters ..." << std::endl;
    CameraParam params;
    params.auto_exposure = true;           // 自动曝光
    params.exposure = 3247;               // 曝光时间
    params.gain = 4096.0;                 // 增益
    params.brightness = 0.0;              // 亮度
    params.contrast = 50.0;               // 对比度
    params.sharpness = 50.0;              // 锐度
    params.laser_power = 100;             // 激光强度
    params.min_distance = 0.0;            // 最小距离
    params.post_processing_sharpening = 3.0; // 后处理锐化
    params.pre_processing_sharpening = 5.0; // 预处理锐化
    params.noise_filtering = 4.0;         // 噪声滤波
    params.confidence_threshold = 1.0;    // 信任阈值
    params.enable_rgb = true;             // 彩色图使能
    params.enable_pointcloud = true;      // 点云使能
    params.enable_ir_left = true;         // 左红外图使能
    params.align_method = 1;              // 对齐方式 0: IR, 1: COLOR
    if (!camera.configureParameters(params))
    {
        std::cerr << "    ✘ Failed to configure parameters" << std::endl;
        return -1;
    }

    // 步骤5: 采集一帧
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待相机稳定后,再取图
    std::cout << "🚀 Step 5: Capturing one frame ..." << std::endl;
    std::map<std::string, std::shared_ptr<void>> outputs;
    if (!camera.capture(outputs))
    {
        std::cerr << "    ✘ Capture failed" << std::endl;
        return -1;
    }

    // 生成时间戳用于文件名
    std::string timestamp = camera.getTimestamp();

    // 保存RGB图像
    if (outputs.count("rgb"))
    {
        auto rgb = std::static_pointer_cast<cv::Mat>(outputs["rgb"]);
        cv::Mat rgb_output;
        cv::cvtColor(*rgb, rgb_output, cv::COLOR_BGR2RGB); // BGR to RGB
        std::string rgb_filename = "rgb_" + timestamp + ".png";
        if (cv::imwrite(rgb_filename, *rgb))
        {
            std::cout << "    ✔ Saved RGB image to " << rgb_filename << std::endl;
        }
        else
        {
            std::cerr << "    ✘ Failed to save RGB image" << std::endl;
        }
    }

    // 保存红外图像
    if (outputs.count("ir_left"))
    {
        auto ir = std::static_pointer_cast<cv::Mat>(outputs["ir_left"]);
        std::string ir_filename = "ir_left_" + timestamp + ".png";
        if (cv::imwrite(ir_filename, *ir))
        {
            std::cout << "    ✔ Saved IR image to " << ir_filename << std::endl;
        }
        else
        {
            std::cerr << "    ✘ Failed to save IR image" << std::endl;
        }
    }

    // 保存点云
    if (outputs.count("pointcloud"))
    {
        // auto pc = std::static_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(outputs["pointcloud"]);
        auto pc = std::static_pointer_cast<pcl::PointCloud<pcl::PointXYZRGB>>(outputs["pointcloud"]);
        std::string pc_filename = "pointcloud_" + timestamp + ".pcd";
        if (pcl::io::savePCDFileASCII(pc_filename, *pc) == 0)
        {
            std::cout << "    ✔ Saved pointcloud to " << pc_filename << "，contains " << pc->size() << " points." << std::endl;
        }
        else
        {
            std::cerr << "    ✘ Failed to save pointcloud" << std::endl;
        }
    }

    // 保存点云查找表
    if(outputs.count("pointcloud_lut"))
    {
        PointCloudRegLUTHandler lut_handler;
        auto lut = std::static_pointer_cast<PointCloudRegLUT>(outputs["pointcloud_lut"]);
        if(lut_handler.saveToFile(lut, "lut_" + timestamp + ".txt", false))
        {
            std::cout << "    ✔ Saved pointcloud lookup table to lut_" + timestamp + ".txt" << std::endl;
        }
        else
        {
            std::cerr << "    ✘ Failed to save pointcloud lookup table" << std::endl;
        }
    }

    /*
    // 步骤6: 根据图像坐标以及查找表，查找3D点云
    PointCloudRegLUTHandler lutHandler;
    // 6.1. 加载查找表（PointCloudRegLUT）
    std::string lut_filename = "lut_" + timestamp + ".txt";
    std::shared_ptr<PointCloudRegLUT> pointcloudReg = std::make_shared<PointCloudRegLUT>();
    try {
        *pointcloudReg = lutHandler.loadFromFile(lut_filename, false);
        std::cout << "Lookup table loaded successfully." << std::endl;
    }
    catch (const std::runtime_error& e) {
        std::cerr << "Failed to load lookup table: " << e.what() << std::endl;
        return -1;
    }

    // 6.2. 加载点云
    std::string pc_filename = "pointcloud_" + timestamp + ".pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pc_filename, *pc_out) == -1) {
        std::cerr << "Failed to load point cloud file." << std::endl;
        return -1;
    }
    std::cout << "Point cloud loaded successfully with " << pc_out->size() << " points." << std::endl;

    // 6.3. 查找指定的 2D 坐标对应的 3D 点云
    int x = 520;  // 假设我们要查询的2D图像坐标 (x, y)
    int y = 500;
    pcl::PointXYZRGB point;

    if (lutHandler.getPoint3DFrom2D(x, y, pointcloudReg, pc_out, point)) {
        std::cout << "The 3D point at (" << x << ", " << y << ") is: "
                  << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
    } else {
        std::cerr << "Failed to find the 3D point for the given 2D coordinates." << std::endl;
    }
    */

    // 步骤7: 断开相机
    std::cout << "🚀 Step6: Disconnecting camera ..." << std::endl;
    camera.disconnect();

    return 0;
}