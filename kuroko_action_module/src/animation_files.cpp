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
void AnimationData::setAnimationBlock(const int block_id, const AnimationBlock& block)
{
  blocks[block_id] = block;
}

void AnimationData::removeAnimationBlock(const int block_id)
{
  blocks.erase(block_id);
}

AnimationBlock AnimationData::getAnimationBlock(const int block_id) const
{
  auto it = blocks.find(block_id);
  if (it == blocks.end())
    throw std::out_of_range("Block ID not found: " + std::to_string(block_id));
  return it->second;
}

std::vector<int> AnimationData::getBlockIds() const
{
  std::vector<int> ids;
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

int AnimationData::getStartBlockId() const
{
  for (const auto& pair : blocks)
  {
    const AnimationBlock& block = pair.second;
    if (block.type == "start")
    {
      return block.id;
    }
  }
  throw std::runtime_error("Start block not found");
}

int AnimationData::getBlockIdByName(const std::string& block_name) const
{
  for (const auto& pair : blocks)
  {
    const AnimationBlock& block = pair.second;
    if (block.filename == block_name || block.type == block_name)
    {
      return block.id;
    }
  }
  throw std::runtime_error("Block with name '" + block_name + "' not found");
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
          pose_frame.move_duration = pose_root["time"]["move_duration"].as<double>(0.0);
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
        if (debug_messages_)
          std::cout << "[Wor kspace] Loaded initial_pose.yaml\n";
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
      if (root["layout"] && root["layout"]["block"])
      {
        const auto& block_root = root["layout"]["block"];
        for (const auto& item : block_root)
        {
          std::string block_name = item.first.as<std::string>();
          const auto& node = item.second;

          AnimationBlock block;
          if (node["info"])
          {
            const auto& info = node["info"];
            block.type = info["type"].as<std::string>("");
            block.filename = info["filename"].as<std::string>("");
            block.id = info["id"].as<int>(-1);
          }

          if (node["connection"] && node["connection"]["output"])
          {
            const auto& output = node["connection"]["output"];
            for (const auto& output_pair : output)
            {
              const auto& output_entry = output_pair.second;
              if (output_entry["target"])
              {
                std::string target_block_name = output_entry["target"].as<std::string>();
                if (block_root[target_block_name] && block_root[target_block_name]["info"] &&
                    block_root[target_block_name]["info"]["id"])
                {
                  int target_id = block_root[target_block_name]["info"]["id"].as<int>();
                  block.output_ids.push_back(target_id);
                }
              }
            }
          }

          anim.setAnimationBlock(block.id, block);
          if (debug_messages_)
          {
            std::cout << "[Workspace]   Block Loaded: name='" << block_name << "', id=" << block.id
                      << ", type=" << block.type << ", filename=" << block.filename << "\n";
          }
        }
      }

      // Load initial_frame.yaml
      if (fs::exists(init_frame_file))
      {
        YAML::Node frame_root = YAML::LoadFile(init_frame_file);
        if (frame_root["joints"])
        {
          FrameData init_frame;
          if (frame_root["time"])
          {
            init_frame.move_duration = frame_root["time"]["move_duration"].as<double>(0.0);
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
          if (debug_messages_)
            std::cout << "[Workspace] Loaded initial_frame.yaml for animation: " << anim_name << "\n";
        }
      }

      // Load frame/if/switch files for all blocks
      std::string frames_dir = (entry.path() / "frames").string();
      std::string if_dir = (entry.path() / "conditions" / "if").string();
      std::string switch_dir = (entry.path() / "conditions" / "switch").string();

      for (const auto& pair : anim.blocks)
      {
        const auto& block = pair.second;
        const std::string& fname = block.filename;

        try
        {
          if (block.type == "frame")
          {
            std::string frame_path = (fs::path(frames_dir) / (fname + ".yaml")).string();
            // std::cout << "[Workspace][DEBUG] Looking for frame file: " << frame_path << std::endl;
            if (fs::exists(frame_path))
            {
              YAML::Node node = YAML::LoadFile(frame_path);
              FrameData frame;
              if (node["time"])
              {
                frame.move_duration = node["time"]["move_duration"].as<double>(0.0);
                frame.wait_duration = node["time"]["wait_duration"].as<double>(0.0);
              }
              if (node["joints"])
              {
                for (const auto& joint_pair : node["joints"])
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
                  if (node["speed_scale"] && node["speed_scale"][joint_name])
                    joint.speed_scale = node["speed_scale"][joint_name].as<double>(1.0);
                  frame.joints[joint_name] = joint;
                }
              }
              anim.setFrameData(fname, frame);
              if (debug_messages_)
                std::cout << "[Workspace]   Loaded frame file: " << fname << "\n";
            }
            else
            {
              std::cerr << "[Workspace][ERROR] Frame file not found: " << frame_path << std::endl;
            }
          }
          else if (block.type == "if")
          {
            std::string if_path = (fs::path(if_dir) / (fname + ".yaml")).string();
            if (fs::exists(if_path))
            {
              YAML::Node node = YAML::LoadFile(if_path);
              IfData ifdata;
              ifdata.expression = node["expression"].as<std::string>("");
              ifdata.condition = node["condition"].as<std::string>("");
              anim.setIfData(fname, ifdata);
              if (debug_messages_)
                std::cout << "[Workspace]   Loaded if-condition file: " << fname << "\n";
            }
          }
          else if (block.type == "switch")
          {
            std::string switch_path = (fs::path(switch_dir) / (fname + ".yaml")).string();
            if (fs::exists(switch_path))
            {
              YAML::Node node = YAML::LoadFile(switch_path);
              SwitchData sdata;
              sdata.expression = node["expression"].as<std::string>("");
              sdata.condition = node["condition"].as<std::string>("");
              if (node["case"])
              {
                for (const auto& case_pair : node["case"])
                {
                  std::string case_label = case_pair.first.as<std::string>();
                  std::map<std::string, int> target_map;
                  for (const auto& inner : case_pair.second)
                  {
                    std::string label = inner.first.as<std::string>();
                    int target_id = inner.second.as<int>();
                    target_map[label] = target_id;
                  }
                  sdata.cases[case_label] = target_map;
                }
              }
              anim.setSwitchData(fname, sdata);
              if (debug_messages_)
                std::cout << "[Workspace]   Loaded switch-condition file: " << fname << "\n";
            }
          }
        }
        catch (const std::exception& e)
        {
          std::cerr << "[Workspace]   Failed to load block file: " << fname << " — " << e.what() << std::endl;
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
