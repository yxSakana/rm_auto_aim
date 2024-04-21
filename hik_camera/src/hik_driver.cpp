#include <hik_camera/hik_driver.h>

#include <sstream>
#include <map>

#include <rclcpp/logging.hpp>

namespace hik_camera {
const static std::map<unsigned int, std::string> kErrorMess {
            {0x80000000, "错误或无效的句柄"},
            {0x80000001, "不支持的功能"},
            {0x80000004, "错误的参数"},
            {0x80000006, "资源申请失败"},
            {0x80000007, "无数据"},
            {0x800000FF, "未知的错误"},
            {0x80000100, "参数非法"},
            {0x80000107, "超时"},
            {0x800000FF, "未知的错误"},
            {0x00008001, "设备断开连接"},
            {0x8000000D, "没有可输出的缓存"},
            {0x80000106, "节点访问条件有误"}
};

HikDriver::HikDriver() {
    connectDevice(0);
}

HikDriver::HikDriver(int index) {
    connectDevice(index);
}

HikDriver::~HikDriver() {
    disconnectDevice();
}

int HikDriver::getDeviceNumber() {
    return check(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE,&m_devices), "EnumDevices")
                ? m_devices.nDeviceNum: 0;
}

bool HikDriver::connectDevice(int index) {
    m_is_connected = 
        check(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &m_devices), "EnumDevices") &&
        static_cast<unsigned int>(index) < m_devices.nDeviceNum &&
        check(MV_CC_CreateHandle(&m_handle, m_devices.pDeviceInfo[index]), "CreateHandle") &&
        check(MV_CC_OpenDevice(m_handle), "OpenDevice") &&
        check(MV_CC_StartGrabbing(m_handle), "StartGrabbing");
    if (m_is_connected) m_index = index;
    return m_is_connected;
}

void HikDriver::disconnectDevice() {
    MV_CC_StopGrabbing(m_handle);
    MV_CC_CloseDevice(m_handle);
    MV_CC_DestroyHandle(m_handle);
}

HikDeviceInfo HikDriver::getDeviceInfo(int index) const {
    if ((unsigned int)index >= m_devices.nDeviceNum) return {};

    MV_CC_DEVICE_INFO* driver_info = m_devices.pDeviceInfo[index];
    if (driver_info == nullptr) return {};

    HikDeviceInfo info;
    if (driver_info->nTLayerType == MV_GIGE_DEVICE) {
        auto cip = driver_info->SpecialInfo.stGigEInfo.nCurrentIp;
        unsigned int nIp1 = ((cip & 0xff000000) >> 24);
        unsigned int nIp2 = ((cip & 0x00ff0000) >> 16);
        unsigned int nIp3 = ((cip & 0x0000ff00) >> 8);
        unsigned int nIp4 = (cip & 0x000000ff);
        std::ostringstream oss;
        oss << nIp1 << "." << nIp2 << "." << nIp3 << "." << nIp4;
        info.ip = oss.str().c_str();
        info.type = "Gige";
        info.name = reinterpret_cast<char*>(driver_info->SpecialInfo.stGigEInfo.chModelName);
    } else if (driver_info->nTLayerType == MV_USB_DEVICE) {
        info.type = "USB";
        info.name = reinterpret_cast<char*>(driver_info->SpecialInfo.stUsb3VInfo.chModelName);
        info.ip = "";
    } else {
        info.type = "Unknown";
    }
    return info;
}

std::string HikDriver::getDeviceParamInfo() const {
    std::ostringstream oss;
    auto v_et = getExposureTimer();
    auto v_g = getGain();
    auto device_info = getDeviceInfo(m_index);

    oss << "\nParam Info: \n"
        << "    index: " << m_index << "; " << to_string(device_info) << "\n"
        << "    Exposure Time: " << to_string(v_et) << "\n"
        << "    Gain: " << to_string(v_g) << "\n";

    return oss.str();
}

void HikDriver::setExposureTime(float time) {
    check(MV_CC_SetFloatValue(m_handle, "ExposureTime", time), "setExposureTimer");
}

MVCC_FLOATVALUE HikDriver::getExposureTimer() const {
    MVCC_FLOATVALUE val;
    check(MV_CC_GetFloatValue(m_handle, "ExposureTime", &val), "getExposureTime");
    return val;
}

void HikDriver::setGain(float gain) {
    check(MV_CC_SetFloatValue(m_handle, "Gain", gain), "setGain");
}

MVCC_FLOATVALUE HikDriver::getGain() const {
    MVCC_FLOATVALUE val;
    check(MV_CC_GetFloatValue(m_handle, "Gain", &val), "getGaom");
    return val;
}

bool HikDriver::readImageData(MV_FRAME_OUT& buff, const unsigned int& timeout_ms) {
    return check(MV_CC_GetImageBuffer(m_handle, &buff, timeout_ms), "readImageData[Height]");
}

bool HikDriver::check(const Code& code, const char* mess) const {
    if (code == MV_OK) return true;

    std::string error_mess;
    try {
        error_mess = kErrorMess.at(code);
    } catch (const std::out_of_range& a) {
        error_mess = "";
    }
    RCLCPP_ERROR(rclcpp::get_logger("hik_camera"), "%s: [%#x] => %s", mess, code, error_mess.c_str());
    return false;
};

std::string to_string(const HikDeviceInfo& info) {
    std::ostringstream oss;
    oss << "[name: " << info.name << "; type: " << info.type << "; ip: " << info.ip << "]";
    return oss.str();
}

std::string to_string(const MVCC_FLOATVALUE& mv_float) {
    std::ostringstream oss;
    oss << mv_float.fCurValue << "(" << mv_float.fMin << " ~ " << mv_float.fMax << ")";
    return oss.str();
}
}
