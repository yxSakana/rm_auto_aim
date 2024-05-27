#ifndef ARMOR_AUTO_AIN_BUFF_DETECTOR__H
#define ARMOR_AUTO_AIN_BUFF_DETECTOR__H

#include <vector>

#include <opencv2/opencv.hpp>

namespace armor_auto_aim {
enum class FanBoardState {
    Unlight,
    Target,
    Activated
};

struct FanBoard {
    int id;
    cv::RotatedRect region;
    FanBoardState state;
};
using FanBoards    = std::vector<FanBoard>;
using BuffFanBoard = std::array<FanBoard, 5>;
using BuffR        = cv::RotatedRect;

struct Buff {
    BuffFanBoard fan_boards;
    BuffR        r;
};

class DetectorStateMachine {
    enum UpgradState {
        Unknown,
        Updating,
        TmpLoss,
        Confirmed
    };
public:
    DetectorStateMachine() =default;

    void setConfirmThreshold(int v) { m_confirme_threshold = v; }

    void setAbortThreshold(int v) { m_abort_threshold = v; }

    void init() { m_state = Unknown; }

    void update(bool condition);

    void confirmed() { m_state = Unknown; };

    bool isUpdating() const { return m_state == Updating || m_state == TmpLoss; }

    bool isConfirmed() const { return m_state == Confirmed; }
private:
    UpgradState m_state = Unknown;
    int m_accord_count = 0;
    int m_loss_count = 0;

    int m_confirme_threshold = 10;
    int m_abort_threshold = 10;
};

class FanBoardDetectorStateMachine {
public:
    FanBoardDetectorStateMachine() =default;

    void init(const FanBoard& target);

    void update(const FanBoards& fan_boards);

    void confirmed() { m_state_machine.confirmed(); }

    bool isUpdating() const { return m_state_machine.isUpdating(); }

    bool isConfirmed() const { return m_state_machine.isConfirmed(); }

    std::shared_ptr<FanBoard> getUpdatingTarget() const { return m_updating_target; }
private:
    DetectorStateMachine m_state_machine;
    std::shared_ptr<FanBoard> m_updating_target;
};

class FanBoardDetector {
public:
    struct Paramters {
        double min_area;
        double max_area;
        double min_ratio;
        double max_ratio;
        double distance_from_r;
    };

    FanBoardDetector(const Paramters& params): m_params(params) {}

    FanBoards detect(const Buff& buff, const cv::Mat& binary_img);
private:
    Paramters m_params;
    FanBoards m_last_results;
    FanBoardDetectorStateMachine m_state_machine;

    int getId(const FanBoard& fan_board, const BuffFanBoard& buff_fan_board);

    bool isFanBoard(const cv::RotatedRect& rrect, const BuffR& r);
};

class RDetector {
public:
    struct Parameters {
        double min_area;
        double max_area;
        double min_ratio;
        double max_ratio;
        // double distance_from_center;
    };

    RDetector(const Parameters& parameters): m_param(parameters) {}

    BuffR detect(const cv::Mat& binary_img);
private:
    Parameters m_param;

    bool isR(const cv::RotatedRect& rrect);
};

class BuffDetector {
public:
    struct PreproessParameters {
        int r_threshold;
        int g_threshold;
        int b_threshold;

        double inner;
        double outer;
    };

    struct Parameters {
        PreproessParameters         preprocess;
        FanBoardDetector::Paramters fan_board;
        RDetector::Parameters       r;
    };

    BuffDetector(const Parameters& parameters);

    cv::Mat preprocessImage(const cv::Mat& src);

    Buff detect(const cv::Mat& binary_img);

    std::shared_ptr<FanBoard> getTarget() const { return m_target; }

    cv::Mat getBinaryImg() const { return m_binary_img; }

    cv::Mat getCirclrImg() const { return m_circle_img; }
private:
    Parameters                        m_param;
    std::shared_ptr<FanBoardDetector> m_fan_detector;
    std::shared_ptr<RDetector>        m_r_detector;
    Buff                              m_last_buff;
    std::shared_ptr<FanBoard>         m_target;
    // Debug
    cv::Mat m_binary_img;
    cv::Mat m_circle_img;

    BuffFanBoard createBuffFanBoard(const FanBoard& fan, const BuffR& r);
};

inline bool isVaild(const cv::Mat& src, const cv::Rect& r) {
    return (0 <= r.x && 0 <= r.width && r.x + r.width <= src.cols &&
        0 <= r.y && 0 <= r.height && r.y + r.height <= src.rows);
}
}

#endif
