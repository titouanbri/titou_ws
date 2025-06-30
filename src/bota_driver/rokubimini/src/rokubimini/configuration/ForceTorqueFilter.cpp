#include <rokubimini/configuration/ForceTorqueFilter.hpp>

namespace rokubimini
{
namespace configuration
{
ForceTorqueFilter::ForceTorqueFilter(const uint16_t sincFilterSize, const uint8_t chopEnable, const uint8_t skipEnable,
                                     const uint8_t fastEnable)
  : sincFilterSize_(sincFilterSize), chopEnable_(chopEnable), skipEnable_(skipEnable), fastEnable_(fastEnable)
{
}

bool ForceTorqueFilter::load(const std::string& key, const NodeHandlePtr& nh)
{
  bool success = false;
  std::string local_key;
  local_key = key + "/sinc_filter_size";
  if (nh->hasParam(local_key))
  {
    int sinc_filter_size;
    nh->getParam(local_key, sinc_filter_size);
    sincFilterSize_ = static_cast<uint16_t>(sinc_filter_size);
    success = true;
  }
  local_key = key + "/chop_enable";
  if (nh->hasParam(local_key))
  {
    bool chop_enable;
    nh->getParam(local_key, chop_enable);
    chopEnable_ = static_cast<uint8_t>(chop_enable);
    success = true;
  }
  local_key = key + "/fir_disable";
  if (nh->hasParam(local_key))
  {
    bool skip_enable;
    nh->getParam(local_key, skip_enable);
    skipEnable_ = static_cast<uint8_t>(skip_enable);
    success = true;
  }
  local_key = key + "/fast_enable";
  if (nh->hasParam(local_key))
  {
    bool fast_enable;
    nh->getParam(local_key, fast_enable);
    fastEnable_ = static_cast<uint8_t>(fast_enable);
    success = true;
  }
  return success;
}

void ForceTorqueFilter::print() const
{
  ROS_INFO_STREAM("sinc_filter_size_: " << static_cast<unsigned int>(sincFilterSize_));
  ROS_INFO_STREAM("chopEnable_: " << static_cast<unsigned int>(chopEnable_));
  ROS_INFO_STREAM("skipEnable_: " << static_cast<unsigned int>(skipEnable_));
  ROS_INFO_STREAM("fastEnable_: " << static_cast<unsigned int>(fastEnable_));
}

}  // namespace configuration
}  // namespace rokubimini