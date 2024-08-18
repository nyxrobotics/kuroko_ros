#ifndef KUROKO_WALKING_PARAMETER_H_
#define KUROKO_WALKING_PARAMETER_H_

class WalkingTimeParameter
{
public:
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

private:
  double periodtime_;
  double dsp_ratio_;
  double ssp_ratio_;
  double x_swap_periodtime_;
  double x_move_periodtime_;
  double y_swap_periodtime_;
  double y_move_periodtime_;
  double z_swap_periodtime_;
  double z_move_periodtime_;
  double a_move_periodtime_;
  double ssp_time_;
  double ssp_time_start_l_;
  double ssp_time_end_l_;
  double ssp_time_start_r_;
  double ssp_time_end_r_;
  double phase1_time_;
  double phase2_time_;
  double phase3_time_;
};

class WalkingMovementParameter
{
private:
};

class WalkingBalanceParameter
{
};

#endif /* KUROKO_WALKING_PARAMETER_H_ */
