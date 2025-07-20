#pragma once

#include "scilab_optimization/scilab_optimization.h"
#include <vector>
#include <ros/ros.h>
#include "kuroko_online_walking_module_msgs/PreviewRequest.h"
#include "kuroko_online_walking_module_msgs/PreviewResponse.h"
#include "kuroko_online_walking_module_msgs/GetPreviewMatrix.h"

class KurokoOptimization
{
public:
  KurokoOptimization();
  ~KurokoOptimization();

  bool getPreviewMatrixCallback(kuroko_online_walking_module_msgs::GetPreviewMatrix::Request& req,
                                kuroko_online_walking_module_msgs::GetPreviewMatrix::Response& res);

private:
  robotis_framework::ScilabOptimization scilab_optimization_;
  bool calcPreviewParam(double control_cycle, double lipm_height);

  double P_row_, P_col_;
  std::vector<double_t> P_;
  double K_row_, K_col_;
  std::vector<double_t> K_;
};
