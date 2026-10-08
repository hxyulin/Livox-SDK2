//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include "parse_cfg_file.h"
#include "base/logging.h"

#include <fstream>
#include <limits>
#include <map>
#include <string>

namespace livox {
namespace lidar {

using nlohmann::json;

const std::map<std::string, LivoxLidarDeviceType> dev_type_map = {
  {"HAP",     kLivoxLidarTypeIndustrialHAP},
  {"MID360",  kLivoxLidarTypeMid360},
  {"Mid360s", kLivoxLidarTypeMid360s},
  {"Avia2",   kLivoxLidarTypeAvia2},
  {"Mid360l", kLivoxLidarTypeMid360l}
};

namespace {

// Reads a required member that must be an unsigned integer fitting in 32 bits.
bool GetUint(const json& object, const char* key, const char* scope, uint32_t& value) {
  auto it = object.find(key);
  if (it == object.end() || !it->is_number_unsigned() ||
      it->get<uint64_t>() > std::numeric_limits<uint32_t>::max()) {
    LOG_ERROR("Parse {} failed, has not {} member or {} is not uint.", scope, key, key);
    return false;
  }
  value = it->get<uint32_t>();
  return true;
}

template <typename T>
bool GetUintAs(const json& object, const char* key, const char* scope, T& out) {
  uint32_t value = 0;
  if (!GetUint(object, key, scope, value)) {
    return false;
  }
  out = static_cast<T>(value);
  return true;
}

} // namespace

ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {}

bool ParseCfgFile::Parse(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
                         std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr,
                         std::shared_ptr<LivoxLidarLoggerCfg>& lidar_logger_cfg_ptr,
                         std::shared_ptr<LivoxLidarSdkFrameworkCfg>& sdk_framework_cfg_ptr) {
  std::ifstream config_file(path_, std::ios::binary);
  if (!config_file) {
    LOG_INFO("Parse lidar config failed, can not open json config file!");
    return false;
  }
  const json doc = json::parse(config_file, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    LOG_ERROR("Parse lidar config failed, parse the config file has error!");
    return false;
  }

  lidars_cfg_ptr.reset(new std::vector<LivoxLidarCfg>());
  custom_lidars_cfg_ptr.reset(new std::vector<LivoxLidarCfg>());
  lidar_logger_cfg_ptr.reset(new LivoxLidarLoggerCfg());
  sdk_framework_cfg_ptr.reset(new LivoxLidarSdkFrameworkCfg());

  auto master_sdk = doc.find("master_sdk");
  if (master_sdk != doc.end()) {
    if (!master_sdk->is_boolean()) {
      LOG_ERROR("set master/slave sdk error");
      return false;
    }
    sdk_framework_cfg_ptr->master_sdk = master_sdk->get<bool>();
    if (sdk_framework_cfg_ptr->master_sdk) {
      LOG_INFO("set master/slave sdk to master sdk");
    } else {
      LOG_INFO("set master/slave sdk to slave sdk");
    }
  } else {
    LOG_INFO("set master/slave sdk to master sdk by default");
    sdk_framework_cfg_ptr->master_sdk = true;
  }

  auto log_path = doc.find("lidar_log_path");
  auto log_enable = doc.find("lidar_log_enable");
  if (log_enable != doc.end()) {
    if (!log_enable->is_boolean()) {
      LOG_ERROR("Lidar log enable data type is error");
      return false;
    }
    lidar_logger_cfg_ptr->lidar_log_enable = log_enable->get<bool>();

    if (!GetUintAs(doc, "lidar_log_cache_size_MB", "json file", lidar_logger_cfg_ptr->lidar_log_cache_size)) {
      return false;
    }

    if (log_path == doc.end() || !log_path->is_string()) {
      LOG_ERROR("Parse json file failed, has not lidar_log_path member or lidar_log_path is not string");
      return false;
    }
    lidar_logger_cfg_ptr->lidar_log_path = log_path->get<std::string>();
    LOG_INFO("Lidar log cfg, lidar_log_enable:{}, lidar_log_cache_size_MB:{}, lidar_log_path:{}",
        lidar_logger_cfg_ptr->lidar_log_enable, lidar_logger_cfg_ptr->lidar_log_cache_size,
        lidar_logger_cfg_ptr->lidar_log_path.c_str());
  } else {
    lidar_logger_cfg_ptr->lidar_log_enable = false;
    lidar_logger_cfg_ptr->lidar_log_cache_size = 0;
    lidar_logger_cfg_ptr->lidar_log_path = "./"; // TODO Executable program path
    if (log_path != doc.end() && log_path->is_string()) {
      lidar_logger_cfg_ptr->lidar_log_path = log_path->get<std::string>();
    }
    LOG_INFO("Livox lidar logger disable.");
  }

  // Parse in the same fixed order as before, not the map's key order.
  for (const char* name : {"HAP", "MID360", "Mid360s", "Avia2", "Mid360l"}) {
    auto object = doc.find(name);
    if (object == doc.end() || !object->is_object()) {
      continue;
    }
    if (!ParseLidarCfg(*object, dev_type_map.at(name), lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
      return false;
    }
  }
  return true;
}

bool ParseCfgFile::ParseLidarCfg(const json &object, const uint8_t& device_type, std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr, std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) {
  auto host_net_info = object.find("host_net_info");
  if (host_net_info != object.end() && host_net_info->is_array()) {
    if (!ParseNewLidarCfg(object, device_type, lidars_cfg_ptr, custom_lidars_cfg_ptr)) {
      LOG_ERROR("Parse hap lidar new cfg failed.");
      return false;
    }
  } else if (host_net_info != object.end() && host_net_info->is_object()) {
    if (!ParseOldLidarCfg(object, device_type, lidars_cfg_ptr)) {
      LOG_ERROR("Parse hap lidar old cfg failed.");
      return false;
    }
  } else {
    LOG_ERROR("Parse lidar net info failed, has not host_net_info member or host_net_info is not object or arry.");
    return false;
  }
  return true;
}

bool ParseCfgFile::ParseNewLidarCfg(const json &object, const uint8_t& device_type, std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr, std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr) {
  for (const json& host_net_info_object : object.at("host_net_info")) {
    if (!host_net_info_object.is_object()) {
      LOG_ERROR("Parse host net info failed, host_net_info entry is not object.");
      return false;
    }
    auto lidar_ip_arr = host_net_info_object.find("lidar_ip");
    if (lidar_ip_arr == host_net_info_object.end() || !lidar_ip_arr->is_array()) {
      LivoxLidarCfg lidar_cfg;

      if (!ParseTypeLidarCfg(object, host_net_info_object, device_type, lidar_cfg)) {
        return false;
      }

      lidars_cfg_ptr->push_back(std::move(lidar_cfg));
      continue;
    }

    for (const json& lidar_ip : *lidar_ip_arr) {
      LivoxLidarCfg lidar_cfg;

      if (!lidar_ip.is_string()) {
        LOG_ERROR("Parse lidar ip failed, has not lidar_ip member or lidar_ip is not object.");
        return false;
      }
      lidar_cfg.lidar_net_info.lidar_ipaddr = lidar_ip.get<std::string>();

      if (!ParseTypeLidarCfg(object, host_net_info_object, device_type, lidar_cfg)) {
        return false;
      }

      custom_lidars_cfg_ptr->push_back(std::move(lidar_cfg));
    }
  }

  return true;
}

bool ParseCfgFile::ParseOldLidarCfg(const json &object, const uint8_t& device_type, std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr) {
  LivoxLidarCfg lidar_cfg;
  if (!ParseTypeLidarCfg(object, object.at("host_net_info"), device_type, lidar_cfg)) {
    return false;
  }
  lidars_cfg_ptr->push_back(std::move(lidar_cfg));
  return true;
}

bool ParseCfgFile::ParseTypeLidarCfg(const json &object, const json &host_net_info_object, const uint8_t& device_type, LivoxLidarCfg& lidar_cfg) {
  lidar_cfg.device_type = device_type;
  if (!ParseLidarNetInfo(object, lidar_cfg.lidar_net_info)) {
    LOG_ERROR("Parse hap lidar net info failed.");
    return false;
  }
  if (!ParseHostNetInfo(host_net_info_object, lidar_cfg.host_net_info)) {
    LOG_ERROR("Parse host net info failed.");
    return false;
  }
  return true;
}

bool ParseCfgFile::ParseLidarNetInfo(const json &object, LivoxLidarNetInfo& lidar_net_info) {
  auto lidar_net_info_object = object.find("lidar_net_info");
  if (lidar_net_info_object == object.end() || !lidar_net_info_object->is_object()) {
    LOG_ERROR("Parse lidar net info failed, has not lidar_net_info member or lidar_net_info is not object.");
    return false;
  }
  const char* scope = "lidar net info";
  return GetUintAs(*lidar_net_info_object, "cmd_data_port", scope, lidar_net_info.cmd_data_port) &&
         GetUintAs(*lidar_net_info_object, "push_msg_port", scope, lidar_net_info.push_msg_port) &&
         GetUintAs(*lidar_net_info_object, "point_data_port", scope, lidar_net_info.point_data_port) &&
         GetUintAs(*lidar_net_info_object, "imu_data_port", scope, lidar_net_info.imu_data_port) &&
         GetUintAs(*lidar_net_info_object, "log_data_port", scope, lidar_net_info.log_data_port);
}

bool ParseCfgFile::ParseHostNetInfo(const json &host_net_info_object, HostNetInfo& host_net_info) {
  auto host_ip = host_net_info_object.find("host_ip");
  auto cmd_data_ip = host_net_info_object.find("cmd_data_ip");
  if (host_ip == host_net_info_object.end() && cmd_data_ip == host_net_info_object.end()) {
    LOG_ERROR("Parse host net info failed, has not host_ip or cmd_data_ip.");
    return false;
  }
  if (host_ip != host_net_info_object.end() && !host_ip->is_string()) {
    LOG_ERROR("Parse host net info failed, host_ip is not string.");
    return false;
  }
  if (cmd_data_ip != host_net_info_object.end() && !cmd_data_ip->is_string()) {
    LOG_ERROR("Parse host net info failed, cmd_data_ip is not string.");
    return false;
  }

  // host_ip takes precedence over cmd_data_ip when both are present.
  if (host_ip != host_net_info_object.end()) {
    host_net_info.host_ip = host_ip->get<std::string>();
  } else {
    host_net_info.host_ip = cmd_data_ip->get<std::string>();
  }

  auto multicast_ip = host_net_info_object.find("multicast_ip");
  if (multicast_ip != host_net_info_object.end()) {
    if (!multicast_ip->is_string()) {
      LOG_ERROR("Parse host net info failed, has not multicast_ip or multicast_ip is not string.");
      return false;
    }
    host_net_info.multicast_ip = multicast_ip->get<std::string>();
  } else {
    host_net_info.multicast_ip = "";
  }

  const char* scope = "host net info";
  return GetUintAs(host_net_info_object, "cmd_data_port", scope, host_net_info.cmd_data_port) &&
         GetUintAs(host_net_info_object, "push_msg_port", scope, host_net_info.push_msg_port) &&
         GetUintAs(host_net_info_object, "point_data_port", scope, host_net_info.point_data_port) &&
         GetUintAs(host_net_info_object, "imu_data_port", scope, host_net_info.imu_data_port) &&
         GetUintAs(host_net_info_object, "log_data_port", scope, host_net_info.log_data_port);
}

} // namespace lidar
} // namespace livox
