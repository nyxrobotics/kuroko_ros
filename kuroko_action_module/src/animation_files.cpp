#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include "kuroko_action_module/animation_files.h"

#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

namespace animation_system
{
void AnimationData::setAnimationBlock(const std::string& block_id, const AnimationBlock& block)
{
  blocks[block_id] = block;
}

void AnimationData::removeAnimationBlock(const std::string& block_id)
{
  blocks.erase(block_id);
}

AnimationBlock AnimationData::getAnimationBlock(const std::string& block_id) const
{
  auto it = blocks.find(block_id);
  if (it == blocks.end())
    throw std::out_of_range("Block ID not found: " + block_id);
  return it->second;
}

std::vector<std::string> AnimationData::getBlockIds() const
{
  std::vector<std::string> ids;
  for (const auto& pair : blocks)
    ids.push_back(pair.first);
  return ids;
}
void AnimationData::setInitialFrameData(const FrameData& frame_data)
{
  initial_frame_data = frame_data;
}

FrameData AnimationData::getInitialFrameData() const
{
  return initial_frame_data;
}

void AnimationData::setFrameData(const std::string& filename, const FrameData& frame_data)
{
  frames[filename] = frame_data;
}

void AnimationData::removeFrameData(const std::string& filename)
{
  frames.erase(filename);
}

FrameData AnimationData::getFrameData(const std::string& filename) const
{
  auto it = frames.find(filename);
  if (it == frames.end())
    throw std::out_of_range("Frame file not found: " + filename);
  return it->second;
}

std::vector<std::string> AnimationData::getFrameFilenames()
{
  std::vector<std::string> names;
  for (const auto& pair : frames)
    names.push_back(pair.first);
  return names;
}

void AnimationData::setIfData(const std::string& filename, const IfData& if_data)
{
  if_conditions[filename] = if_data;
}

void AnimationData::removeIfData(const std::string& filename)
{
  if_conditions.erase(filename);
}

IfData AnimationData::getIfData(const std::string& filename)
{
  auto it = if_conditions.find(filename);
  if (it == if_conditions.end())
    throw std::out_of_range("If condition not found: " + filename);
  return it->second;
}

std::vector<std::string> AnimationData::getIfFilenames()
{
  std::vector<std::string> names;
  for (const auto& pair : if_conditions)
    names.push_back(pair.first);
  return names;
}

void AnimationData::setSwitchData(const std::string& filename, const SwitchData& switch_data)
{
  switch_conditions[filename] = switch_data;
}

void AnimationData::removeSwitchData(const std::string& filename)
{
  switch_conditions.erase(filename);
}

SwitchData AnimationData::getSwitchData(const std::string& filename)
{
  auto it = switch_conditions.find(filename);
  if (it == switch_conditions.end())
    throw std::out_of_range("Switch condition not found: " + filename);
  return it->second;
}

std::vector<std::string> AnimationData::getSwitchFilenames()
{
  std::vector<std::string> names;
  for (const auto& pair : switch_conditions)
    names.push_back(pair.first);
  return names;
}
void Workspace::loadWorkspace(const std::string& workspace_path)
{
  animations.clear();
  initial_pose_data = FrameData();  // Reset

  // Load workspace-level initial_pose.yaml
  std::string initial_pose_path = (fs::path(workspace_path) / "initial_pose.yaml").string();
  if (fs::exists(initial_pose_path))
  {
    try
    {
      YAML::Node pose_root = YAML::LoadFile(initial_pose_path);
      if (pose_root["joints"])
      {
        FrameData pose_frame;
        if (pose_root["time"])
        {
          pose_frame.move_duration = pose_root["time"]["move_duration"].as<double>(2.0);
          pose_frame.wait_duration = pose_root["time"]["wait_duration"].as<double>(0.0);
        }
        for (const auto& joint_pair : pose_root["joints"])
        {
          std::string joint_name = joint_pair.first.as<std::string>();
          const auto& joint_node = joint_pair.second;
          JointData joint;
          joint.position = joint_node["position"].as<double>(0.0);
          joint.enable = joint_node["enable"].as<bool>(true);
          if (joint_node["pid"])
            joint.pid = joint_node["pid"].as<std::vector<double>>();
          if (joint_node["feedback"])
            joint.feedback = joint_node["feedback"].as<std::string>();
          if (pose_root["speed_scale"] && pose_root["speed_scale"][joint_name])
            joint.speed_scale = pose_root["speed_scale"][joint_name].as<double>(1.0);
          pose_frame.joints[joint_name] = joint;
        }
        initial_pose_data = pose_frame;
        std::cout << "[Workspace] Loaded initial_pose.yaml\n";
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "[Workspace] Failed to load initial_pose.yaml: " << e.what() << std::endl;
    }
  }

  // Load animation folders
  for (const auto& entry : fs::directory_iterator(workspace_path))
  {
    if (!fs::is_directory(entry))
      continue;

    std::string anim_name = entry.path().filename().string();
    std::string yaml_file = (entry.path() / "animation.yaml").string();
    std::string init_frame_file = (entry.path() / "initial_frame.yaml").string();

    if (!fs::exists(yaml_file))
      continue;

    try
    {
      YAML::Node root = YAML::LoadFile(yaml_file);
      AnimationData anim;

      // Load blocks
      if (root["blocks"])
      {
        for (const auto& item : root["blocks"])
        {
          std::string id = item.first.as<std::string>();
          const auto& node = item.second;
          AnimationBlock block;
          block.type = node["type"].as<std::string>();
          block.filename = node["filename"].as<std::string>();
          if (node["output_ids"])
            block.output_ids = node["output_ids"].as<std::vector<std::string>>();
          anim.setAnimationBlock(id, block);
        }
      }

      // Load initial_frame.yaml if present
      if (fs::exists(init_frame_file))
      {
        YAML::Node frame_root = YAML::LoadFile(init_frame_file);
        if (frame_root["joints"])
        {
          FrameData init_frame;
          if (frame_root["time"])
          {
            init_frame.move_duration = frame_root["time"]["move_duration"].as<double>(2.0);
            init_frame.wait_duration = frame_root["time"]["wait_duration"].as<double>(0.0);
          }
          for (const auto& joint_pair : frame_root["joints"])
          {
            std::string joint_name = joint_pair.first.as<std::string>();
            const auto& joint_node = joint_pair.second;
            JointData joint;
            joint.position = joint_node["position"].as<double>(0.0);
            joint.enable = joint_node["enable"].as<bool>(true);
            if (joint_node["pid"])
              joint.pid = joint_node["pid"].as<std::vector<double>>();
            if (joint_node["feedback"])
              joint.feedback = joint_node["feedback"].as<std::string>();
            if (frame_root["speed_scale"] && frame_root["speed_scale"][joint_name])
              joint.speed_scale = frame_root["speed_scale"][joint_name].as<double>(1.0);
            init_frame.joints[joint_name] = joint;
          }
          anim.setInitialFrameData(init_frame);
          std::cout << "[Workspace] Loaded initial_frame.yaml for animation: " << anim_name << "\n";
        }
      }

      animations[anim_name] = anim;
    }
    catch (const std::exception& e)
    {
      std::cerr << "[Workspace] Failed to load animation: " << yaml_file << "\nError: " << e.what() << std::endl;
    }
  }
}

std::vector<std::string> Workspace::getAnimationNames()
{
  std::vector<std::string> names;
  for (const auto& pair : animations)
    names.push_back(pair.first);
  return names;
}

const AnimationData& Workspace::getAnimationData(const std::string& name) const
{
  auto it = animations.find(name);
  if (it == animations.end())
    throw std::out_of_range("Animation not found: " + name);
  return it->second;
}

FrameData Workspace::getInitialPoseData()
{
  return initial_pose_data;
}

void Workspace::setInitialPoseData(const FrameData& frame_data)
{
  initial_pose_data = frame_data;
}

}  // namespace animation_system
