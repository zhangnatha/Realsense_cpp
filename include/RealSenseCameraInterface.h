/** @file RealSenseCameraInterface.h
 *  @brief RealSense相机接口定义
 *  @date 2025.09.05
 */

#ifndef S_DEVICE_REALSENSECAMERA_H
#define S_DEVICE_REALSENSECAMERA_H

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** @enum CameraStatus
 *  @brief 相机状态枚举
 */
enum CameraStatus
{
    CAMERA_STATUS_INIT,           // 相机初始化状态
    CAMERA_STATUS_CONNECTED,      // 相机已连接
    CAMERA_STATUS_LOST_CONNECTION, // 相机连接丢失
    CAMERA_STATUS_CAPTURE_SUCCESS, // 捕获成功
    CAMERA_STATUS_CAPTURE_FAILED   // 捕获失败
};

/** @struct CameraParam
 *  @brief 相机参数结构体
 */
struct CameraParam
{
    bool auto_exposure = true;     // 自动曝光
    int exposure = 3247;           // 曝光时间
    double gain = 4096.0;          // 增益
    double brightness = 0.0;       // 亮度
    double contrast = 50.0;        // 对比度
    double sharpness = 50.0;       // 锐度
    int laser_power = 100;         // 激光强度
    double min_distance = 0.0;     // 最小距离
    double confidence_threshold = 1.0; // 信任阈值
    double post_processing_sharpening = 3.0; // 后处理锐化
    double pre_processing_sharpening = 5.0; // 预处理锐化
    double noise_filtering = 4.0;   // 噪声过滤
    bool enable_rgb = true;         // 彩色图使能
    bool enable_pointcloud = true;  // 点云使能
    bool enable_ir_left = true;     // 左红外图使能
    int align_method = 0;           // 对齐方式 0: IR, 1: COLOR
    bool enable_pointcloudLUT = true; // 点云LUT查找表使能
};

/** @struct CameraInfo
 *  @brief 相机信息结构体
 */
struct CameraInfo
{
    std::string serial_number;    // 相机序列号
    std::string vendor;           // 相机供应商
    std::string manufacturer;     // 相机制造商信息（固件版本）
};

/** @class RealSenseCameraInterface
 *  @brief RealSense相机接口类
 */
class RealSenseCameraInterface
{
public:
    /** @brief 构造函数，初始化相机接口 */
    RealSenseCameraInterface();

    /** @brief 析构函数，释放相机资源 */
    ~RealSenseCameraInterface();

    /** @brief 扫描可用的RealSense相机 */
    std::vector<CameraInfo> scanCameras();

    /** @brief 选择指定序列号的相机 */
    bool selectCamera(const std::string& sn);

    /** @brief 连接到选定的相机 */
    bool connect();

    /** @brief 配置相机参数 */
    bool configureParameters(const CameraParam& params);

    /** @brief 捕获一帧数据（RGB、红外、点云） */
    bool capture(std::map<std::string, std::shared_ptr<void>>& outputs);

    /** @brief 断开相机连接 */
    bool disconnect();

    /** @brief 获取相机当前状态 */
    CameraStatus getStatus();

    /** @brief 获取时间戳字符串，用于生成文件名 */
    std::string getTimestamp();

private:
    /** @brief 执行相机硬件复位 */
    bool hardwareReset();

    std::unique_ptr<rs2::pipeline> pipe_; // 相机数据管道
    rs2::config cfg_;                     // 相机配置对象
    rs2::context ctx_;                    // 相机上下文
    std::vector<rs2::sensor> sensors_;    // 相机传感器列表
    rs2::device device_;                  // 当前选中的相机设备
    std::string selected_sn_;             // 选中的相机序列号
    std::string selected_name_;           // 选中的相机型号名称
    CameraParam params_;                  // 相机参数
    CameraStatus status_ = CAMERA_STATUS_INIT; // 相机当前状态
    bool is_capturing_ = false;           // 是否正在捕获数据
    bool callback_disconnected_ = false;   // 相机断开连接回调标志
};

#endif // S_DEVICE_REALSENSECAMERA_H