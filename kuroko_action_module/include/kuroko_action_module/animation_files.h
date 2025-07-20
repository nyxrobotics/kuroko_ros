#ifndef ANIMATION_FILES_H_
#define ANIMATION_FILES_H_

#include <string>
#include <vector>
#include <map>

namespace animation_system
{
struct JointData
{
  double position = 0.0;
  double speed_scale = 1.0;
  bool enable = true;
  std::vector<double> pid = { 0.0, 0.0, 0.0 };
  std::string feedback;
};

struct FrameData
{
  double move_duration = 1.0;
  double wait_duration = 0.0;
  std::map<std::string, JointData> joints;  // <joint_name, JointData>
};

struct IfData
{
  std::string expression;
  std::string condition;
};

struct SwitchData
{
  std::string expression;
  std::string condition;
  std::map<std::string, std::map<std::string, int>> cases;
};

struct AnimationBlock
{
  std::string type;
  std::string filename;
  int id;
  std::vector<int> output_ids;
};

class AnimationData
{
public:
  std::map<int, AnimationBlock> blocks;  // <block_id, AnimationBlock>
  FrameData initial_frame_data;
  std::map<std::string, FrameData> frames;              // <filename, FrameData>
  std::map<std::string, IfData> if_conditions;          // <filename, IfData>
  std::map<std::string, SwitchData> switch_conditions;  // <filename, SwitchData>

  void setAnimationBlock(const int block_id, const AnimationBlock& block);
  void removeAnimationBlock(const int block_id);
  AnimationBlock getAnimationBlock(const int block_id) const;
  std::vector<int> getBlockIds() const;

  void setInitialFrameData(const FrameData& frame);
  FrameData getInitialFrameData() const;

  void setFrameData(const std::string& filename, const FrameData& frame_data);
  void removeFrameData(const std::string& filename);
  FrameData getFrameData(const std::string& filename) const;
  std::vector<std::string> getFrameFilenames();

  void setIfData(const std::string& filename, const IfData& if_data);
  void removeIfData(const std::string& filename);
  IfData getIfData(const std::string& filename);
  std::vector<std::string> getIfFilenames();

  void setSwitchData(const std::string& filename, const SwitchData& switch_data);
  void removeSwitchData(const std::string& filename);
  SwitchData getSwitchData(const std::string& filename);
  std::vector<std::string> getSwitchFilenames();

  int getStartBlockId() const;
  int getBlockIdByName(const std::string& block_name) const;
};

class Workspace
{
public:
  std::map<std::string, AnimationData> animations;  // <animation_name, AnimationData>
  FrameData initial_pose_data;

  void loadWorkspace(const std::string& workspace_path);
  std::vector<std::string> getAnimationNames();
  const AnimationData& getAnimationData(const std::string& name) const;
  FrameData getInitialPoseData();
  void setInitialPoseData(const FrameData& frame_data);
};

}  // namespace animation_system

#endif  // ANIMATION_FILES_H_
