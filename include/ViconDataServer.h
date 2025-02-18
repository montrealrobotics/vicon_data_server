#ifndef VICON_DATA_SERVER_H
#define VICON_DATA_SERVER_H

#include "IDataStreamClientBase.h"
#include <string>
#include <Eigen/Dense>

namespace ViconDataStreamSDK
{
namespace CPP
{
  inline std::string ResultToString(Result::Enum result)
  {
    switch (result)
    {
      case Result::Unknown:                        return "Unknown";
      case Result::NotImplemented:                 return "NotImplemented";
      case Result::Success:                        return "Success";
      case Result::InvalidHostName:                return "InvalidHostName";
      case Result::InvalidMulticastIP:             return "InvalidMulticastIP";
      case Result::ClientAlreadyConnected:         return "ClientAlreadyConnected";
      case Result::ClientConnectionFailed:         return "ClientConnectionFailed";
      case Result::ServerAlreadyTransmittingMulticast: return "ServerAlreadyTransmittingMulticast";
      case Result::ServerNotTransmittingMulticast: return "ServerNotTransmittingMulticast";
      case Result::NotConnected:                   return "NotConnected";
      case Result::NoFrame:                        return "NoFrame";
      case Result::InvalidIndex:                   return "InvalidIndex";
      case Result::InvalidCameraName:              return "InvalidCameraName";
      case Result::InvalidSubjectName:             return "InvalidSubjectName";
      case Result::InvalidSegmentName:             return "InvalidSegmentName";
      case Result::InvalidMarkerName:              return "InvalidMarkerName";
      case Result::InvalidDeviceName:              return "InvalidDeviceName";
      case Result::InvalidDeviceOutputName:        return "InvalidDeviceOutputName";
      case Result::InvalidLatencySampleName:       return "InvalidLatencySampleName";
      case Result::CoLinearAxes:                   return "CoLinearAxes";
      case Result::LeftHandedAxes:                 return "LeftHandedAxes";
      case Result::HapticAlreadySet:               return "HapticAlreadySet";
      case Result::EarlyDataRequested:             return "EarlyDataRequested";
      case Result::LateDataRequested:              return "LateDataRequested";
      case Result::InvalidOperation:               return "InvalidOperation";
      case Result::NotSupported:                   return "NotSupported";
      case Result::ConfigurationFailed:            return "ConfigurationFailed";
      case Result::NotPresent:                     return "NotPresent";
      case Result::ArgumentOutOfRange:             return "ArgumentOutOfRange";
    }
    return "Unrecognized Error Code";
  }


  std::string to_json_array(const Eigen::VectorXd& vec) {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i < vec.size() - 1) {
            ss << ", ";  // Add commas between elements
        }
    }
    ss << "]";
    return ss.str();
  }

} // namespace CPP
} // namespace ViconDataStreamSDK

#endif // VICON_DATA_SERVER_H
