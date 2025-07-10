#include "CameraRS.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <pcl/io/pcd_io.h>

namespace s {
namespace device {

#define USER_LOG std::cerr
#define Log_debug std::cerr
#define Log_error std::cerr
#define Log_warn std::cerr
#define DEBUG_RS true

// 实现 C++11 兼容的 clamp 函数
template<typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

CameraRS::CameraRS() {
    // 移除 camera_flag，初始化其他成员
}

CameraRS::~CameraRS() {
    disconnect();
}

void CameraRS::setSerialNumber(const std::string& serial) {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    _camsn = serial;
}

int CameraRS::initialize() {
    // std::lock_guard<std::recursive_mutex> lock(_mutex);
    _startConnect = true;
    if (_isConnect) {
        Log_error << "[CameraRS][0x03210008]camera init failed, Realsense already connected" << std::endl;
        return 0;
    }

    Log_debug << "[CameraRS]Starting initialization..." << std::endl;

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    auto devices = ctx.query_devices();
    int devices_cnt = devices.size();

    Log_debug << "[CameraRS]Found " << devices_cnt << " devices" << std::endl;

    if (devices_cnt <= 0) {
        Log_error << "[CameraRS][0x03210009]CameraRS init failed, no RealSense cameras found" << std::endl;
        return -1;
    }

    if (DEBUG_RS) std::cout << "Device information:" << std::endl;
    std::vector<std::string> serials;
    std::vector<std::string> names;
    for (int i = 0; i < devices_cnt; i++) {
        rs2::device cur_dev = devices[i];
        std::string name = cur_dev.get_info(RS2_CAMERA_INFO_NAME);
        std::string serial = cur_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string version = cur_dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);

        Log_debug << "[Camera]Found device: ModelName=" << name << ", SerialNumber=" << serial << ", DeviceVersion=" << version << std::endl;
        serials.push_back(serial);
        names.push_back(name);
        if (serial == _camsn) {
            _id = i;
            _camsn = serial;
        }
    }

    if (_id == -1) {
        Log_error << "[CameraRS]No camera found with serial number: " << _camsn << std::endl;
        return -1;
    }

    if (DEBUG_RS) std::cout << "Sensor count for first device: " << devices[0].query_sensors().size() << std::endl;
    Log_debug << "[CameraRS]Connecting to device with serial number: " << _camsn << std::endl;

    dev = devices[_id];
    sensors = dev.query_sensors();
    selected_name_ = names[_id];

    if (DEBUG_RS) std::cout << "[CameraRS]Connected to device: " << selected_name_ << std::endl;

    try {
        rs2::config cfg;
        cfg.disable_all_streams();
        cfg.enable_device(_camsn);

        // 配置流，注意 USB 带宽问题
        if (selected_name_ == "Intel RealSense L515") {
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
            cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 15);
            cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 15);
        } else if (selected_name_ == "Intel RealSense D435" || selected_name_ == "Intel RealSense D435I") {
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
            cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 15);
            cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
        } else {
            Log_warn << "[CameraRS]Unsupported camera model: " << selected_name_ << ", using default configuration" << std::endl;
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);
        }

        if (DEBUG_RS) std::cout << "Starting pipeline..." << std::endl;
        _pipelines[_camsn] = rs2::pipeline(ctx);
        _pipelines[_camsn].start(cfg);
        _colorizers[_camsn] = rs2::colorizer();
    } catch (const rs2::error& e) {
        Log_error << "[CameraRS][0x0321000D]RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "): " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        Log_error << "[CameraRS][0x0321000E]Standard exception: " << e.what() << std::endl;
        return -1;
    }

    _callback_disconnected = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 确保设备稳定

    ctx.set_devices_changed_callback([this](rs2::event_information info) {
        if (info.was_removed(dev)) {
            _callback_disconnected = true;
            Log_debug << "[CameraRS]Device disconnected callback received" << std::endl;
            if (DEBUG_RS) std::cout << "---CAMERA DISCONNECTED---" << std::endl;
        }
    });

    return 0;
}

int CameraRS::connect() {
    if (_startConnect) {
        Log_debug << "[CameraRS]Camera is connecting, please wait..." << std::endl;
        return -1;
    }
    if (initialize() == 0) {
        _startConnect = false;
        Log_debug << "[CameraRS]Camera connected successfully" << std::endl;
        _isConnect = true;
        downloadCameraParams();
        return 0;
    } else {
        _startConnect = false;
        _isConnect = false;
        Log_error << "[CameraRS][0x0321000A]Camera connection failed" << std::endl;
        return -1;
    }
}

int CameraRS::disconnect() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    _startConnect = false;
    _callback_disconnected = true;
    if (_isConnect) {
        // 使用 C++11 兼容的循环替代结构化绑定
        for (std::map<std::string, rs2::pipeline>::iterator it = _pipelines.begin(); it != _pipelines.end(); ++it) {
            it->second.stop();
        }
        _pipelines.clear();
        _colorizers.clear();
        _isConnect = false;
    } else {
        Log_debug << "[CameraRS]Camera is already disconnected" << std::endl;
    }
    _isFinishCapture = true;
    Log_debug << "[CameraRS]Camera disconnected successfully" << std::endl;
    return 0;
}

int CameraRS::capture() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    if (!_isConnect) {
        Log_error << "[CameraRS]Camera not connected, cannot capture" << std::endl;
        _captureStatus = -1;
        return -1;
    }
    if (!_isFinishCapture) {
        Log_warn << "[CameraRS]Previous capture not finished" << std::endl;
        return 0;
    }
    Log_debug << "[CameraRS]Capture begin" << std::endl;

    if (parseFrame() != 1) {
        Log_error << "[CameraRS][0x0321000B]Capture failed, parse frame error" << std::endl;
        _captureStatus = -1;
        return -1;
    }
    Log_debug << "[CameraRS]Capture successful" << std::endl;
    _captureStatus = 1;
    return 1;
}

bool CameraRS::isCameraConnect() {
    return !_callback_disconnected && _isConnect;
}

void CameraRS::setCameraStatus(bool isConnect) {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    _isConnect = isConnect;
}

int CameraRS::getCameraCaptureStatus() {
    return _captureStatus;
}

void CameraRS::setCameraCaptureStatus(int status) {
    _captureStatus = status;
}

bool CameraRS::isCameraDisconnected() {
    return !_isConnect || _callback_disconnected;
}

int CameraRS::setCameraParamters(const CameraPara& paras) {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    _cam_para = paras;
    return downloadCameraParams();
}

CameraRS::CameraPara CameraRS::getCameraParas() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    return _cam_para;
}

static const std::map<rs2_option, std::string> option_to_string = {
    {RS2_OPTION_BACKLIGHT_COMPENSATION, "RS2_OPTION_BACKLIGHT_COMPENSATION"},
    {RS2_OPTION_BRIGHTNESS, "RS2_OPTION_BRIGHTNESS"},
    {RS2_OPTION_CONTRAST, "RS2_OPTION_CONTRAST"},
    {RS2_OPTION_EXPOSURE, "RS2_OPTION_EXPOSURE"},
    {RS2_OPTION_GAIN, "RS2_OPTION_GAIN"},
    {RS2_OPTION_GAMMA, "RS2_OPTION_GAMMA"},
    {RS2_OPTION_HUE, "RS2_OPTION_HUE"},
    {RS2_OPTION_SATURATION, "RS2_OPTION_SATURATION"},
    {RS2_OPTION_SHARPNESS, "RS2_OPTION_SHARPNESS"},
    {RS2_OPTION_WHITE_BALANCE, "RS2_OPTION_WHITE_BALANCE"},
    {RS2_OPTION_ENABLE_AUTO_EXPOSURE, "RS2_OPTION_ENABLE_AUTO_EXPOSURE"},
    {RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, "RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE"},
    {RS2_OPTION_VISUAL_PRESET, "RS2_OPTION_VISUAL_PRESET"},
    {RS2_OPTION_LASER_POWER, "RS2_OPTION_LASER_POWER"},
    {RS2_OPTION_EMITTER_ENABLED, "RS2_OPTION_EMITTER_ENABLED"},
    {RS2_OPTION_FRAMES_QUEUE_SIZE, "RS2_OPTION_FRAMES_QUEUE_SIZE"},
    {RS2_OPTION_TOTAL_FRAME_DROPS, "RS2_OPTION_TOTAL_FRAME_DROPS"},
    {RS2_OPTION_POWER_LINE_FREQUENCY, "RS2_OPTION_POWER_LINE_FREQUENCY"},
    {RS2_OPTION_ASIC_TEMPERATURE, "RS2_OPTION_ASIC_TEMPERATURE"},
    {RS2_OPTION_ERROR_POLLING_ENABLED, "RS2_OPTION_ERROR_POLLING_ENABLED"},
    {RS2_OPTION_PROJECTOR_TEMPERATURE, "RS2_OPTION_PROJECTOR_TEMPERATURE"},
    {RS2_OPTION_OUTPUT_TRIGGER_ENABLED, "RS2_OPTION_OUTPUT_TRIGGER_ENABLED"},
    {RS2_OPTION_DEPTH_UNITS, "RS2_OPTION_DEPTH_UNITS"},
    {RS2_OPTION_STEREO_BASELINE, "RS2_OPTION_STEREO_BASELINE"},
    {RS2_OPTION_INTER_CAM_SYNC_MODE, "RS2_OPTION_INTER_CAM_SYNC_MODE"},
    {RS2_OPTION_EMITTER_ON_OFF, "RS2_OPTION_EMITTER_ON_OFF"},
    {RS2_OPTION_GLOBAL_TIME_ENABLED, "RS2_OPTION_GLOBAL_TIME_ENABLED"},
    {RS2_OPTION_EMITTER_ALWAYS_ON, "RS2_OPTION_EMITTER_ALWAYS_ON"},
    {RS2_OPTION_HDR_ENABLED, "RS2_OPTION_HDR_ENABLED"},
    {RS2_OPTION_SEQUENCE_NAME, "RS2_OPTION_SEQUENCE_NAME"},
    {RS2_OPTION_SEQUENCE_SIZE, "RS2_OPTION_SEQUENCE_SIZE"},
    {RS2_OPTION_SEQUENCE_ID, "RS2_OPTION_SEQUENCE_ID"}
};

int CameraRS::downloadCameraParams() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    try {
        // 延迟确保设备稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (DEBUG_RS) {
            for (std::vector<rs2::sensor>::iterator sensor = sensors.begin(); sensor != sensors.end(); ++sensor) {
                Log_debug << "[CameraRS]传感器类型: " << (sensor->is<rs2::depth_sensor>() ? "深度" : "彩色") << std::endl;
                for (int i = 0; i < RS2_OPTION_COUNT; i++) {
                    rs2_option option = static_cast<rs2_option>(i);
                    if (sensor->supports(option)) {
                        try {
                            std::map<rs2_option, std::string>::const_iterator it = option_to_string.find(option);
                            std::string option_name = (it != option_to_string.end()) ? it->second : "未知选项";
                            Log_debug << "[CameraRS]支持的选项: " << option_name << std::endl;
                            float value = sensor->get_option(option);
                            Log_debug << "[CameraRS]选项值: " << option_name << " = " << value << std::endl;
                        } catch (const rs2::error& e) {
                            Log_error << "[CameraRS]获取选项 " << i << " 失败: " << e.what() << std::endl;
                        }
                    }
                }
            }
        }

        for (std::vector<rs2::sensor>::iterator sensor = sensors.begin(); sensor != sensors.end(); ++sensor) {
            if (sensor->is<rs2::depth_sensor>()) {
                rs2::depth_sensor depth_sensor = sensor->as<rs2::depth_sensor>();
                if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                    try {
                        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
                        Log_debug << "[CameraRS]设置 RS2_OPTION_EMITTER_ENABLED 为 1" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } catch (const rs2::error& e) {
                        Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_EMITTER_ENABLED 失败: " << e.what() << std::endl;
                    }
                }

                if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
                    try {
                        rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
                        float value = clamp(static_cast<float>(_cam_para.laser_power), range.min, range.max);
                        depth_sensor.set_option(RS2_OPTION_LASER_POWER, value);
                        Log_debug << "[CameraRS]设置 RS2_OPTION_LASER_POWER 为 " << value << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } catch (const rs2::error& e) {
                        Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_LASER_POWER 失败: " << e.what() << std::endl;
                    }
                }
            } else if (sensor->is<rs2::color_sensor>()) {
                rs2::color_sensor color_sensor = sensor->as<rs2::color_sensor>();
                if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                    try {
                        color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, _cam_para.auto_exposure);
                        Log_debug << "[CameraRS]设置 RS2_OPTION_ENABLE_AUTO_EXPOSURE 为 " << _cam_para.auto_exposure << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } catch (const rs2::error& e) {
                        Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_ENABLE_AUTO_EXPOSURE 失败: " << e.what() << std::endl;
                    }
                }

                if (!_cam_para.auto_exposure) {
                    if (color_sensor.supports(RS2_OPTION_EXPOSURE)) {
                        try {
                            rs2::option_range range = color_sensor.get_option_range(RS2_OPTION_EXPOSURE);
                            float value = clamp(_cam_para.exposure, range.min, range.max);
                            color_sensor.set_option(RS2_OPTION_EXPOSURE, value);
                            Log_debug << "[CameraRS]设置 RS2_OPTION_EXPOSURE 为 " << value << std::endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        } catch (const rs2::error& e) {
                            Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_EXPOSURE 失败: " << e.what() << std::endl;
                        }
                    }

                    if (color_sensor.supports(RS2_OPTION_GAIN)) {
                        try {
                            rs2::option_range range = color_sensor.get_option_range(RS2_OPTION_GAIN);
                            float value = clamp(_cam_para.gain, range.min, range.max);
                            color_sensor.set_option(RS2_OPTION_GAIN, value);
                            Log_debug << "[CameraRS]设置 RS2_OPTION_GAIN 为 " << value << std::endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        } catch (const rs2::error& e) {
                            Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_GAIN 失败: " << e.what() << std::endl;
                        }
                    }
                }

                if (color_sensor.supports(RS2_OPTION_BRIGHTNESS)) {
                    try {
                        rs2::option_range range = color_sensor.get_option_range(RS2_OPTION_BRIGHTNESS);
                        float value = clamp(_cam_para.brightness, range.min, range.max);
                        color_sensor.set_option(RS2_OPTION_BRIGHTNESS, value);
                        Log_debug << "[CameraRS]设置 RS2_OPTION_BRIGHTNESS 为 " << value << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } catch (const rs2::error& e) {
                        Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_BRIGHTNESS 失败: " << e.what() << std::endl;
                    }
                }

                if (color_sensor.supports(RS2_OPTION_CONTRAST)) {
                    try {
                        rs2::option_range range = color_sensor.get_option_range(RS2_OPTION_CONTRAST);
                        float value = clamp(_cam_para.contrast, range.min, range.max);
                        color_sensor.set_option(RS2_OPTION_CONTRAST, value);
                        Log_debug << "[CameraRS]设置 RS2_OPTION_CONTRAST 为 " << value << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } catch (const rs2::error& e) {
                        Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_CONTRAST 失败: " << e.what() << std::endl;
                    }
                }

                if (color_sensor.supports(RS2_OPTION_SHARPNESS)) {
                    try {
                        rs2::option_range range = color_sensor.get_option_range(RS2_OPTION_SHARPNESS);
                        float value = clamp(_cam_para.sharpness, range.min, range.max);
                        color_sensor.set_option(RS2_OPTION_SHARPNESS, value);
                        Log_debug << "[CameraRS]设置 RS2_OPTION_SHARPNESS 为 " << value << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } catch (const rs2::error& e) {
                        Log_error << "[CameraRS][0x0321000F]设置 RS2_OPTION_SHARPNESS 失败: " << e.what() << std::endl;
                    }
                }
            }
        }
    } catch (const rs2::error& e) {
        Log_error << "[CameraRS][0x0321000F]设置相机参数失败: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        Log_error << "[CameraRS][0x0321000F]设置相机参数时发生标准异常: " << e.what() << std::endl;
        return -1;
    }

    Log_debug << "[CameraRS]相机参数设置完成" << std::endl;
    return 0;
}

int CameraRS::parseFrame() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    _isFinishCapture = false;

    if (DEBUG_RS) std::cout << "Parsing frame..." << std::endl;

    rs2::frameset frames;
    if (!_pipelines[_camsn].poll_for_frames(&frames)) {
        _isFinishCapture = true;
        Log_debug << "[CameraRS]No frames available" << std::endl;
        return 0;
    }

    rs2::frameset aligned_frames = align_to_depth.process(frames);
    rs2::depth_frame depth = aligned_frames.get_depth_frame();
    rs2::video_frame color = aligned_frames.get_color_frame();

    if (_cam_para.enable_rgb && color) {
        int width = color.get_width();
        int height = color.get_height();
        int bytes_per_pixel = color.get_bytes_per_pixel();
        Log_debug << "[Camera]RGB resolution: w=" << width << ", h=" << height << ", w*h=" << width * height << std::endl;

        char* data = (char*)malloc(width * height * bytes_per_pixel);
        if (!data) {
            Log_error << "[CameraRS][0x03210011]Memory allocation failed for RGB image" << std::endl;
            _isFinishCapture = true;
            return -1;
        }
        memcpy(data, color.get_data(), width * height * bytes_per_pixel);
        cv::Mat cvImg(height, width, CV_8UC3, data);
        cv::cvtColor(cvImg, cvImg, cv::COLOR_RGB2BGR);
        cv::imwrite("rgb_image.jpg", cvImg);
        free(data);
        Log_debug << "[CameraRS]RGB image saved as rgb_image.jpg" << std::endl;
    }

    if (_cam_para.enable_pointcloud && depth) {
        rs2::pointcloud rs_pc;
        rs_pc.map_to(color);
        rs2::points points = rs_pc.calculate(depth);
        Log_debug << "[Camera]Depth resolution: w=" << depth.get_width() << ", h=" << depth.get_height() << ", w*h=" << depth.get_width() * depth.get_height() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
        auto vertices = points.get_vertices();
        for (size_t i = 0; i < points.size(); i++) {
            if (vertices[i].z > 0) {
                pointcloud->push_back(pcl::PointXYZ(1000 * vertices[i].x, 1000 * vertices[i].y, 1000 * vertices[i].z));
            }
        }
        pcl::io::savePCDFileASCII("pointcloud.pcd", *pointcloud);
        Log_debug << "[CameraRS]Point cloud saved as pointcloud.pcd" << std::endl;
    }

    _isFinishCapture = true;
    return 1;
}

void CameraRS::setOutPuts(const std::string& key, std::shared_ptr<void> data) {
    // 简化为保存到文件，实际应用中可扩展
}

std::vector<std::pair<std::string, std::string>> CameraRS::scanDevices() {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    std::cout << "检测到设备数量: " << devices.size() << std::endl;
    std::vector<std::pair<std::string, std::string>> result;
    for (size_t i = 0; i < devices.size(); ++i) {
        rs2::device dev = devices[i];
        std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        result.push_back({serial, name});
    }
    return result;
}

} // namespace device
} // namespace s
