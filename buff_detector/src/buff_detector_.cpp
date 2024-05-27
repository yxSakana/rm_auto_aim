#include <buff_detector/buff_detector_.h>

#include <rclcpp/logging.hpp>

#include <buff_detector/iou_compute.h>

namespace armor_auto_aim {
// DetectorStateMachine {{{
void DetectorStateMachine::update(bool condition) {
    if (m_state == Confirmed) return;

    if (condition) {
        ++m_accord_count, m_loss_count = 0;
        if (m_state == Unknown || m_state == TmpLoss)
            m_state = Updating;
        if (m_state == Updating)
            m_state = m_accord_count > m_confirme_threshold? Confirmed: Updating;
    } else {
        ++m_loss_count, m_accord_count = 0;
        if (m_state == Updating)
            m_state = TmpLoss;
        else if (m_state == TmpLoss)
            m_state = m_loss_count > m_abort_threshold ? Unknown : TmpLoss;
    }
}
// }}} DetectorStateMachine

// FanDetectorStateMachine {{{
void FanBoardDetectorStateMachine::init(const FanBoard& target) {
    m_updating_target = std::make_shared<FanBoard>(target);
    m_state_machine.init();
}

void FanBoardDetectorStateMachine::update(const FanBoards& fan_boards) {
    bool is_matched = false;
    double max_iou = DBL_MIN;
    for (const auto& fan_board: fan_boards) {
        auto val = iou::IoU(m_updating_target->region, fan_board.region);
        if (val > 0.5 && val > max_iou) {
            is_matched = true;
            max_iou = val;
        }
    }
    m_state_machine.update(is_matched);
}
// }}} FanDetectorStateMachine

// FanBoardDetector {{{
FanBoards FanBoardDetector::detect(const Buff& buff, const cv::Mat& binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    FanBoards detected_fan_boards;
    FanBoard fan_board;
    for (const auto& contour: contours) {
        cv::RotatedRect rrect = cv::minAreaRect(contour);
        if (isVaild(binary_img, rrect.boundingRect()) && isFanBoard(rrect, buff.r)) {
            int id = getId(fan_board, buff.fan_boards);
            if (id == -1) continue;
            fan_board.id     = id;
            fan_board.region = rrect,
            fan_board.state  = buff.fan_boards.empty() ?
                                FanBoardState::Unlight :
                                buff.fan_boards[fan_board.id].state;
            detected_fan_boards.emplace_back(fan_board);
        }
    }

    for (auto& last_fan: m_last_results) {
        double max_iou = DBL_MIN;
        for (const auto& fan: detected_fan_boards) {
            double val = iou::IoU(fan.region, last_fan.region);
            if (val > 0.5 && val > max_iou) {
                last_fan = fan;
            }
        }
    }
    if (detected_fan_boards.size() > m_last_results.size()) {
        if (m_state_machine.isUpdating()) {
            m_state_machine.update(detected_fan_boards);
        } else if (m_state_machine.isConfirmed()) {
            m_last_results.emplace_back(*m_state_machine.getUpdatingTarget());
            m_state_machine.confirmed();
        } else {
            int new_id;
            for (const auto& fan: detected_fan_boards) {
                bool is_new_fan  = true;
                new_id = fan.id;
                for (const auto& last_fan: m_last_results) {
                    if (fan.id == last_fan.id) {
                        is_new_fan = false;
                        break;
                    }
                }
                if (is_new_fan) break;
            }
            m_state_machine.init(buff.fan_boards[new_id]);
        }
    } else if (detected_fan_boards.size() < m_last_results.size()) {
        RCLCPP_WARN(rclcpp::get_logger("fan_board_detector"), "detected fan board < last");
    }

    return m_last_results;
}

int FanBoardDetector::getId(const FanBoard& fan_board, const BuffFanBoard& buff_fan_board) {
    if (buff_fan_board.empty()) return 0;
    int id = -1;
    float max_iou = DBL_MIN;
    for (const auto& fan: buff_fan_board) {
        auto val = iou::IoU(fan.region, fan_board.region);
        if (val > 0.5 && val > max_iou) {
            max_iou = val;
            id = fan.id;
        }
    }
    return id;
}

bool FanBoardDetector::isFanBoard(const cv::RotatedRect& rrect, const BuffR& r) {
    float area = rrect.size.area();
    if (area <= 20) return false;
    float w = r.size.width, h = r.size.height;
    if (w > h) std::swap(w, h);
    float ratio = w / h; // 短/长
    if (m_params.min_area < area && area < m_params.max_area &&
        m_params.min_ratio < ratio && ratio < m_params.max_ratio &&
        cv::norm(r.center - rrect.center) < m_params.distance_from_r)
            return true;
    return false;
}
// }}} FanBoardDetector

// RDetector {{{
BuffR RDetector::detect(const cv::Mat& binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (const auto& contour: contours) {
        cv::RotatedRect rrect = cv::minAreaRect(contour);
        if (isVaild(binary_img, rrect.boundingRect()) && isR(rrect)) {
            return rrect;
        }
    }
    return {};
}

bool RDetector::isR(const cv::RotatedRect& rrect) {
    float area = rrect.size.area();
    if (area <= 20) return false;
    float w = rrect.size.width, h = rrect.size.height;
    if (w > h) std::swap(w, h);
    float ratio = w / h; // 短/长
    if (m_param.min_area  < area && area < m_param.max_area &&
        m_param.min_ratio < ratio && ratio < m_param.max_ratio) {
        return true;
    }
    return false;
}
// }}} RDetector

// BuffDetector {{{
BuffDetector::BuffDetector(const Parameters& parameters)
    : m_param(parameters) {
    m_fan_detector = std::make_shared<FanBoardDetector>(m_param.fan_board);
    m_r_detector   = std::make_shared<RDetector>(m_param.r);
}

cv::Mat BuffDetector::preprocessImage(const cv::Mat& src) {
    cv::Mat binary_img = cv::Mat::zeros(src.size(), CV_8UC1);
    uchar* dst_ptr = binary_img.data;
    uchar* src_ptr = src.data;
    for (; src_ptr != src.data + src.cols * src.rows * 3; dst_ptr++) {
        uchar* b = src_ptr++;
        uchar* g = src_ptr++;
        uchar* r = src_ptr++;
        if (*b >= m_param.preprocess.b_threshold &&
              *g >= m_param.preprocess.g_threshold)
            *dst_ptr = 255;
    }
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, k);

    m_binary_img = binary_img;
    return binary_img;
}

Buff BuffDetector::detect(const cv::Mat& binary_img) {
    Buff result = m_last_buff;
    result.r = m_r_detector->detect(binary_img);
    // Process image to detect FanBoard
    cv::circle(binary_img, result.r.center, m_param.preprocess.inner, cv::Scalar(0), -1);
    cv::circle(binary_img, result.r.center, m_param.preprocess.outer, cv::Scalar(0), 5);
    m_circle_img = binary_img;
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, k);
    // Fanboard
    FanBoards detected_fan_boards = m_fan_detector->detect(result, binary_img);
    if (detected_fan_boards.size() > m_last_buff.fan_boards.size()) {
        result.fan_boards[m_target->id].state = FanBoardState::Activated;
        for (const auto& fan: detected_fan_boards) {
            if (m_last_buff.fan_boards[fan.id].state == FanBoardState::Unlight) {
                m_target = std::make_shared<FanBoard>(fan);
            }
        }
    }
    result.fan_boards = m_target? createBuffFanBoard(*m_target, result.r): BuffFanBoard();
    m_last_buff = result;
    return m_last_buff;
}
// }}} BuffDetector
}
