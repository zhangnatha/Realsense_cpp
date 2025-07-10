#ifndef CAMERA_RS_H
#define CAMERA_RS_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <map>
#include <mutex>
#include <memory>

namespace s {
namespace device {

class CameraRS {
public:
    struct CameraPara {
        bool auto_exposure = true;
        int laser_power = 100; // 默认激光功率
        float exposure = 1000.0f; // 曝光时间 (us)
        float gain = 16.0f; // 增益
        float brightness = 0.0f; // 亮度
        float contrast = 50.0f; // 对比度
        float sharpness = 50.0f; // 锐度
        bool enable_rgb = true;
        bool enable_ir_left = false;
        bool enable_pointcloud = true;
        bool enable_pointReg = false;
    };

    CameraRS();
    ~CameraRS();

    int initialize();
    int connect();
    int disconnect();
    int capture();
    bool isCameraConnect();
    void setCameraStatus(bool isConnect);
    int getCameraCaptureStatus();
    void setCameraCaptureStatus(int status);
    bool isCameraDisconnected();
    int setCameraParamters(const CameraPara& paras);
    CameraPara getCameraParas();
    int downloadCameraParams();
    void setSerialNumber(const std::string& serial); // 新增 setter 方法
    static std::vector<std::pair<std::string, std::string>> scanDevices(); // 新增

private:
    int parseFrame();
    void setOutPuts(const std::string& key, std::shared_ptr<void> data);

    std::recursive_mutex _mutex;
    rs2::context ctx;
    rs2::device dev;
    std::vector<rs2::sensor> sensors;
    std::map<std::string, rs2::pipeline> _pipelines;
    std::map<std::string, rs2::colorizer> _colorizers;
    std::string _camsn;
    std::string selected_name_;
    int _id = -1;
    bool _isConnect = false;
    bool _startConnect = false;
    bool _callback_disconnected = false;
    bool _isFinishCapture = true;
    int _captureStatus = 0;
    CameraPara _cam_para;
};

} // namespace device
} // namespace s

#endif // CAMERA_RS_H
