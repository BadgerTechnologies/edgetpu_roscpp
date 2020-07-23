#pragma once
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/cpp/detection/engine.h"
#include "src/cpp/examples/label_utils.h"
#include "src/cpp/examples/model_utils.h"
#include "edgetpu_roscpp/image_resize.h"

namespace edgetpu_roscpp
{

class EdgeTpuObjectDetection: public jsk_topic_tools::DiagnosticNodelet
{
  public:
    EdgeTpuObjectDetection(): DiagnosticNodelet("EdgeTpuObjectDetection"), model_tensor_shape_(0) {}

  protected:
    // Publishers
    // ros::Publisher target_pos_pub_;
    image_transport::Publisher image_pub_;  // DEBUG

    // Subscribers
    image_transport::Subscriber image_sub_;
    ros::Subscriber cam_info_sub_;

    // Image Transport
    boost::shared_ptr<image_transport::ImageTransport> it_;

    // Coral Detection
    boost::shared_ptr<coral::DetectionEngine> detection_engine_;
    std::vector<int> model_tensor_shape_;
    std::unordered_map<int, std::string> labels_;

    // ROS Params
    int top_k_;
    double score_threshold_;
    bool keep_aspect_ratio_;
    bool image_view_;
    bool verbose_;

    // tf2::Matrix3x3 camera_K_inv_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){}

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
};

} //namespace edgetpu_rospp
