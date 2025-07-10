#include "CameraRS.h"
#include <iostream>
#include <thread>

int main() {
    // 1. 扫描相机
    auto devices = s::device::CameraRS::scanDevices();
    if (devices.empty()) {
        std::cerr << "未检测到任何 RealSense 相机！" << std::endl;
        return -1;
    }
    std::cout << "检测到的相机列表：" << std::endl;
    for (size_t i = 0; i < devices.size(); ++i) {
        std::cout << i << ": 型号=" << devices[i].second << "，序列号=" << devices[i].first << std::endl;
    }

    // 2. 选择相机
    int select_idx = 0;
    if (devices.size() > 1) {
        std::cout << "请输入要连接的相机序号（默认0）: ";
        std::string input;
        std::getline(std::cin, input);
        if (!input.empty()) {
            select_idx = std::stoi(input);
            if (select_idx < 0 || select_idx >= devices.size()) select_idx = 0;
        }
    }
    std::string selected_serial = devices[select_idx].first;
    std::cout << "选择的相机序列号: " << selected_serial << std::endl;

    // 3. 连接相机
    s::device::CameraRS camera;
    camera.setSerialNumber(selected_serial);
    if (camera.connect() != 0) {
        std::cerr << "连接相机失败" << std::endl;
        return -1;
    }

    // 4. 设置参数
    s::device::CameraRS::CameraPara params;
    params.auto_exposure = false;
    params.exposure = 1000.0f;
    params.gain = 16.0f;
    params.brightness = 0.0f;
    params.contrast = 50.0f;
    params.sharpness = 50.0f;
    params.laser_power = 100;
    params.enable_rgb = true;
    params.enable_pointcloud = true;
    params.enable_ir_left = false;
    params.enable_pointReg = false;
    if (camera.setCameraParamters(params) != 0) {
        std::cerr << "设置相机参数失败" << std::endl;
        return -1;
    }

    // 5. 拍照获取图像和点云
    for (int i = 0; i < 5; i++) {
        if (camera.capture() != 1) {
            std::cerr << "拍照失败" << std::endl;
        } else {
            std::cout << "已拍照 " << i + 1 << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 6. 断开相机
    camera.disconnect();
    std::cout << "相机已断开" << std::endl;
    return 0;
}
