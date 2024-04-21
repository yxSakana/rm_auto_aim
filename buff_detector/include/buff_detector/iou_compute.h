#ifndef ARMOR_AUTO_AIM_IOU_COMPUTE_H
#define ARMOR_AUTO_AIM_IOU_COMPUTE_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace armor_auto_aim::iou {
static float intersectionArea(const cv::RotatedRect& r1, const cv::RotatedRect& r2) {
    std::vector<cv::Point2f> intersections;
    auto state = cv::rotatedRectangleIntersection(r1, r2, intersections);
    if (state == cv::INTERSECT_NONE) {
        return 0;
    } else {
        std::vector<cv::Point2f> order_pts;
        cv::convexHull(intersections, order_pts, true);
        return cv::contourArea(order_pts);
    }
}

static float IoU(const cv::RotatedRect& r1, const cv::RotatedRect& r2) {
    float intersection_area = intersectionArea(r1, r2);
    if (!intersection_area) return 0;
    float union_area = r1.size.area() + r2.size.area() - intersection_area;
    return intersection_area / union_area;
}

static float GIoU(const cv::RotatedRect& r1, const cv::RotatedRect& r2) {

}

static float DIoU();

static float CIoU();
}

#endif // ARMOR_AUTO_AIM_IOU_COMPUTE_H
