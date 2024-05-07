#include <hik_camera/hik_driver.h>

#include <iostream>
#include <sstream>
#include <map>
#include <bitset>

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

uint16_t HikDriver::mConnectedDevices = 0;

HikDriver::HikDriver() {
    connectDevice(0);
}

HikDriver::HikDriver(int index) {
    connectDevice(index);
}

HikDriver::HikDriver(const char* device_serial_number) {
    connectDevice(device_serial_number);
}

HikDriver::~HikDriver() {
    disconnectDevice();
}

std::string HikDriver::getDeviceSerialNumber() const {
    MVCC_STRINGVALUE s;
    MV_CC_GetStringValue(m_handle, "DeviceSerialNumber", &s);
    return std::string(s.chCurValue);
}

int HikDriver::getDeviceNumber() {
    return check(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &m_devices), "EnumDevices")
                ? m_devices.nDeviceNum: 0;
}

bool HikDriver::connectDevice(int index) {
    bool is_connected = check(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &m_devices), "EnumDevices") &&
        static_cast<unsigned int>(index) < m_devices.nDeviceNum &&
        check(MV_CC_CreateHandle(&m_handle, m_devices.pDeviceInfo[index]), "CreateHandle") &&
        check(MV_CC_OpenDevice(m_handle), "OpenDevice") &&
        check(MV_CC_StartGrabbing(m_handle), "StartGrabbing");
    m_is_connected = is_connected || m_is_connected;
    if (is_connected) {
        m_index = index;
        mConnectedDevices |= 0x0001 << m_index;
        RCLCPP_INFO(rclcpp::get_logger("hik_camera"), "Usage index \"%d\" connected camera", index);
    } 
    return m_is_connected;
}

bool HikDriver::connectDevice(const char* device_serial_number) {
    if (m_is_connected && !strcmp(device_serial_number, getDeviceSerialNumber().c_str()))
        return true;

    void* handle;
    MVCC_STRINGVALUE s;
    bool is_connected = false;

    check(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &m_devices), "EnumDevices");
    for (int i = 0; i < m_devices.nDeviceNum; ++i) {
        RCLCPP_WARN_EXPRESSION(rclcpp::get_logger("hik_camera"),
            !MV_CC_IsDeviceAccessible(m_devices.pDeviceInfo[i], 1),
            "Hik camera index %d is not accessible", i);
        if (!MV_CC_IsDeviceAccessible(m_devices.pDeviceInfo[i], 1) ||
            (0x0001 << i) & mConnectedDevices) continue;
        
        bool is_opened = check(MV_CC_CreateHandle(&handle, m_devices.pDeviceInfo[i]), "CreateHande") &&
            check(MV_CC_OpenDevice(handle), "OpenDevice") &&
            check(MV_CC_GetStringValue(handle, "DeviceSerialNumber", &s), "DeviceSerialNumber");
        if (is_opened && strlen(s.chCurValue) != 0 &&
                !strcmp(s.chCurValue, device_serial_number)) {
            is_connected = check(MV_CC_StartGrabbing(handle), "StartGrabbing");
            m_is_connected = is_connected || m_is_connected;
            if (is_connected) {
                m_index = i;
                m_handle = std::move(handle);
                mConnectedDevices |= 0x0001 << m_index;
                std::bitset<8> bin(mConnectedDevices);
                RCLCPP_INFO(rclcpp::get_logger("hik_camera"),
                    "Usage device serial number \"%s\" connected camera", device_serial_number);
                return true;
            }
        } 
        if (!is_connected) {
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
        }
    }
    return is_connected;
}

void HikDriver::disconnectDevice() {
    MV_CC_StopGrabbing(m_handle);
    MV_CC_CloseDevice(m_handle);
    MV_CC_DestroyHandle(m_handle);
    mConnectedDevices &= 0x1110 << m_index;
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
        << "    Device Serial Number: " << getDeviceSerialNumber() << "\n"
        << "    Index: " << m_index << "; " << to_string(device_info) << "\n"
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
