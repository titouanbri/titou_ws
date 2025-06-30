#include <rokubimini/calibration/SensorCalibration.hpp>

namespace rokubimini
{
namespace calibration
{
bool SensorCalibration::load(const std::string& key, const NodeHandlePtr& nh)
{
  // Do something
  bool success = false;
  std::string calibration_matrix_key = key + "/calibration_matrix";
  std::string local_key;
  local_key = calibration_matrix_key + "/1_1";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(0, 0));
    success = true;
  }
  local_key = calibration_matrix_key + "/1_2";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(0, 1));
    success = true;
  }
  local_key = calibration_matrix_key + "/1_3";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(0, 2));
    success = true;
  }
  local_key = calibration_matrix_key + "/1_4";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(0, 3));
    success = true;
  }
  local_key = calibration_matrix_key + "/1_5";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(0, 4));
    success = true;
  }
  local_key = calibration_matrix_key + "/1_6";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(0, 5));
    success = true;
  }
  local_key = calibration_matrix_key + "/2_1";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(1, 0));
    success = true;
  }
  local_key = calibration_matrix_key + "/2_2";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(1, 1));
    success = true;
  }
  local_key = calibration_matrix_key + "/2_3";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(1, 2));
    success = true;
  }
  local_key = calibration_matrix_key + "/2_4";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(1, 3));
    success = true;
  }
  local_key = calibration_matrix_key + "/2_5";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(1, 4));
    success = true;
  }
  local_key = calibration_matrix_key + "/2_6";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(1, 5));
    success = true;
  }
  local_key = calibration_matrix_key + "/3_1";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(2, 0));
    success = true;
  }
  local_key = calibration_matrix_key + "/3_2";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(2, 1));
    success = true;
  }
  local_key = calibration_matrix_key + "/3_3";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(2, 2));
    success = true;
  }
  local_key = calibration_matrix_key + "/3_4";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(2, 3));
    success = true;
  }
  local_key = calibration_matrix_key + "/3_5";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(2, 4));
    success = true;
  }
  local_key = calibration_matrix_key + "/3_6";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(2, 5));
    success = true;
  }
  local_key = calibration_matrix_key + "/4_1";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(3, 0));
    success = true;
  }
  local_key = calibration_matrix_key + "/4_2";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(3, 1));
    success = true;
  }
  local_key = calibration_matrix_key + "/4_3";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(3, 2));
    success = true;
  }
  local_key = calibration_matrix_key + "/4_4";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(3, 3));
    success = true;
  }
  local_key = calibration_matrix_key + "/4_5";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(3, 4));
    success = true;
  }
  local_key = calibration_matrix_key + "/4_6";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(3, 5));
    success = true;
  }
  local_key = calibration_matrix_key + "/5_1";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(4, 0));
    success = true;
  }
  local_key = calibration_matrix_key + "/5_2";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(4, 1));
    success = true;
  }
  local_key = calibration_matrix_key + "/5_3";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(4, 2));
    success = true;
  }
  local_key = calibration_matrix_key + "/5_4";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(4, 3));
    success = true;
  }
  local_key = calibration_matrix_key + "/5_5";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(4, 4));
    success = true;
  }
  local_key = calibration_matrix_key + "/5_6";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(4, 5));
    success = true;
  }
  local_key = calibration_matrix_key + "/6_1";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(5, 0));
    success = true;
  }
  local_key = calibration_matrix_key + "/6_2";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(5, 1));
    success = true;
  }
  local_key = calibration_matrix_key + "/6_3";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(5, 2));
    success = true;
  }
  local_key = calibration_matrix_key + "/6_4";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(5, 3));
    success = true;
  }
  local_key = calibration_matrix_key + "/6_5";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(5, 4));
    success = true;
  }
  local_key = calibration_matrix_key + "/6_6";
  if (nh->hasParam(local_key))
  {
    nh->getParam(local_key, calibrationMatrix_(5, 5));
    success = true;
  }
  return success;
}
}  // namespace calibration
}  // namespace rokubimini