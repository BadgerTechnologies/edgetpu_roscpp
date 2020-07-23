#include <edgetpu_roscpp/edgetpu_object_detection.h>

namespace edgetpu_roscpp
{

void EdgeTpuObjectDetection::onInit()
{
  DiagnosticNodelet::onInit();
  /* ros params */
  /** for size filter mode **/
  std::string model_file, label_file;
  pnh_->param("model_file", model_file, std::string(""));
  pnh_->param("label_file", label_file, std::string(""));
  pnh_->param("top_k", top_k_, 2);
  pnh_->param("score_threshold", score_threshold_, 0.3);
  pnh_->param("keep_aspect_ratio", keep_aspect_ratio_, false);
  pnh_->param("verbose", verbose_, false);
  pnh_->param("image_view", image_view_, false);
  always_subscribe_ = true;

  labels_ = coral::ReadLabelFile(label_file);
  detection_engine_ = boost::make_shared<coral::DetectionEngine>(model_file);
  model_tensor_shape_ = detection_engine_->get_input_tensor_shape(); // [1, height, width, 3]
  if(model_tensor_shape_.size() != 4 || model_tensor_shape_.at(0) != 1 || model_tensor_shape_.at(3) != 3)
    throw std::runtime_error("the input tensor shape for classification is not correct");

  if (image_view_)
    image_pub_ = advertiseImage(*pnh_, "detection_result", 1);

  it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);

  onInitPostProcess();
}

void EdgeTpuObjectDetection::subscribe()
{
  image_sub_ = it_->subscribe("image", 1, &EdgeTpuObjectDetection::imageCallback, this);
  cam_info_sub_ = nh_->subscribe("camera_info", 1, &EdgeTpuObjectDetection::cameraInfoCallback, this);
}

void EdgeTpuObjectDetection::unsubscribe()
{
  image_sub_.shutdown();
}

void EdgeTpuObjectDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat src_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;

  cv::Mat resized_img;
  CvSize2D32f ratio;
  resize(src_img,
         cv::Size(model_tensor_shape_.at(2), model_tensor_shape_.at(1)),
         keep_aspect_ratio_,
         resized_img,
         ratio);
  std::vector<uint8_t> input_tensor(
      resized_img.data,
      resized_img.data + (resized_img.cols * resized_img.rows * resized_img.elemSize())
  );

  auto results = detection_engine_->DetectWithInputTensor(input_tensor, score_threshold_, top_k_);

  if(verbose_)
    ROS_INFO_STREAM("Deep detection result:" << results.size());

  // DEBUG
  /* std::cout << "Results: " << results << std::endl; */

  for (auto result : results) {

    // DEBUG
    std::cout << result.DebugString() << std::endl;

    result.corners.xmin *= (src_img.size().width / ratio.width);
    result.corners.xmax *= (src_img.size().width / ratio.width);
    result.corners.ymin *= (src_img.size().height / ratio.height);
    result.corners.ymax *= (src_img.size().height / ratio.height);

    if(verbose_) {
      std::cout << "---------------------------" << std::endl;
      std::cout << labels_[result.label] << std::endl;
      std::cout << "Score: " << result.score << std::endl;
      std::cout << "Box: ["
                << result.corners.xmin << ", "
                << result.corners.ymin << ", "
                << result.corners.xmax << ", "
                << result.corners.ymax << "] "
                << std::endl;
    }

    if(image_view_)
      {
        cv::rectangle(src_img,
                      cv::Point(result.corners.xmin, result.corners.ymin),
                      cv::Point(result.corners.xmax, result.corners.ymax),
                      cv::Scalar(0,0,255), 10);
        cv::putText(src_img, std::to_string(result.score),
                    cv::Point(result.corners.xmin, result.corners.ymin + 30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1, cv::Scalar(0,0,255), 2, 8, false);
      }
  }

  if(image_view_)
    image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, src_img).toImageMsg());
}

} //namespace edgetpu_roscpp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (edgetpu_roscpp::EdgeTpuObjectDetection, nodelet::Nodelet);
