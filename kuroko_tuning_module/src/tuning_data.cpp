#include "kuroko_tuning_module/tuning_data.h"

namespace motion_control
{
TuningData::TuningData()
{
}

TuningData::~TuningData()
{
}

void TuningData::clearData()
{
  joint_name_.clear();

  position_.clear();
  velocity_.clear();
  effort_.clear();

  p_gain_.clear();
  i_gain_.clear();
  d_gain_.clear();
}

}  // namespace motion_control
