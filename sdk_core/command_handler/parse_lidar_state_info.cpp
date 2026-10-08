
#include "parse_lidar_state_info.h"
#include "base/logging.h"
#include "nlohmann/json.hpp"

#include <cstring>
#include <iostream>
#include <sstream>

namespace livox {

namespace lidar {

bool ParseLidarStateInfo::Parse(const CommPacket& packet, std::string& info_str) {
  DirectLidarStateInfo info;
  std::set<ParamKeyName> key_mask;
  
  if (!ParseStateInfo(packet, info, key_mask)) {
    return false;
  }

  LivoxLidarStateInfoToJson(info, key_mask, info_str);
  return true;
}

bool ParseLidarStateInfo::ParseStateInfo(const CommPacket& packet,
                                         DirectLidarStateInfo& info,
                                         std::set<ParamKeyName>& key_mask) {  
  uint16_t offset = 0;
  uint16_t key_num = 0;
  memcpy(&key_num, &packet.data[offset], sizeof(uint16_t));
  offset += sizeof(uint16_t) * 2;  

  for (uint16_t i = 0; i < key_num; ++i) {
    if (offset + sizeof(LivoxLidarKeyValueParam) > packet.data_len) {
      return false;
    }

    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&packet.data[offset];
    offset += sizeof(uint16_t);

    uint16_t val_len = 0;
    memcpy(&val_len, &packet.data[offset], sizeof(uint16_t));
    offset += sizeof(uint16_t);
  
    switch (kv->key) {
      case static_cast<uint16_t>(kKeyPclDataType) :
        key_mask.insert(kKeyPclDataType);
        memcpy(&info.pcl_data_type, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyPatternMode) :
        key_mask.insert(kKeyPatternMode);
        memcpy(&info.pattern_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyDualEmitEn) :
        key_mask.insert(kKeyDualEmitEn);
        memcpy(&info.dual_emit_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyPointSendEn) :
        key_mask.insert(kKeyPointSendEn);
        memcpy(&info.point_send_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLidarIpCfg) :
        key_mask.insert(kKeyLidarIpCfg);
        ParseLidarIpAddr(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyStateInfoHostIpCfg) :
        key_mask.insert(kKeyStateInfoHostIpCfg);
        ParseStateInfoHostIPCfg(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyLidarPointDataHostIpCfg) :
        key_mask.insert(kKeyLidarPointDataHostIpCfg);
        ParsePointCloudHostIpCfg(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyLidarImuHostIpCfg) :
        key_mask.insert(kKeyLidarImuHostIpCfg);
        ParseImuDataHostIpCfg(packet, offset, info);
        break;
      case static_cast<uint16_t>(kKeyCtlHostIpCfg) :
        key_mask.insert(kKeyCtlHostIpCfg);
        ParseIpCfg(packet, offset, info.ctl_host_ipcfg);
        break;
      case static_cast<uint16_t>(kKeyLogHostIpCfg) :
        key_mask.insert(kKeyLogHostIpCfg);
        ParseIpCfg(packet, offset, info.log_host_ipcfg);
        break;
      case static_cast<uint16_t>(kKeyVehicleSpeed) :
        key_mask.insert(kKeyVehicleSpeed);
        memcpy(&info.vehicle_speed, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyEnvironmentTemp) :
        key_mask.insert(kKeyEnvironmentTemp);
        memcpy(&info.environment_temp, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyInstallAttitude) :
        key_mask.insert(kKeyInstallAttitude);
        memcpy(&info.install_attitude, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyBlindSpotSet) :
        key_mask.insert(kKeyBlindSpotSet);
        memcpy(&info.blind_spot_set, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFrameRate) :
        key_mask.insert(kKeyFrameRate);
        memcpy(&info.frame_rate, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFovCfg0) :
        key_mask.insert(kKeyFovCfg0);
        memcpy(&info.fov_cfg0, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFovCfg1) :
        key_mask.insert(kKeyFovCfg1);
        memcpy(&info.fov_cfg1, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFovCfgEn) :
        key_mask.insert(kKeyFovCfgEn);
        memcpy(&info.fov_cfg_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyDetectMode) :
        key_mask.insert(kKeyDetectMode);
        memcpy(&info.detect_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFuncIoCfg) :
        key_mask.insert(kKeyFuncIoCfg);
        memcpy(&info.func_io_cfg, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyWorkMode) :
        key_mask.insert(kKeyWorkMode);
        memcpy(&info.work_tgt_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyGlassHeat) :
        key_mask.insert(kKeyGlassHeat);
        memcpy(&info.glass_heat, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyImuDataEn) :
        key_mask.insert(kKeyImuDataEn);
        memcpy(&info.imu_data_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFusaEn) :
        key_mask.insert(kKeyFusaEn);
        memcpy(&info.fusa_en, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySn) :
        key_mask.insert(kKeySn);
        memcpy(info.sn, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyProductInfo) :
        key_mask.insert(kKeyProductInfo);
        memcpy(info.product_info, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyVersionApp) :
        key_mask.insert(kKeyVersionApp);
        memcpy(info.version_app, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyVersionLoader) :
        key_mask.insert(kKeyVersionLoader);
        memcpy(info.version_loader, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyVersionHardware):
        key_mask.insert(kKeyVersionHardware);
        memcpy(info.version_hardware, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyMac) :
        key_mask.insert(kKeyMac);
        memcpy(info.mac, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyCurWorkState) :
        key_mask.insert(kKeyCurWorkState);
        memcpy(&info.cur_work_state, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyCoreTemp) :
        key_mask.insert(kKeyCoreTemp);
        memcpy(&info.core_temp, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyPowerUpCnt) :
        key_mask.insert(kKeyPowerUpCnt);
        memcpy(&info.powerup_cnt, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLocalTimeNow) :
        key_mask.insert(kKeyLocalTimeNow);
        memcpy(&info.local_time_now, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLastSyncTime) :
        key_mask.insert(kKeyLastSyncTime);
        memcpy(&info.last_sync_time, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyTimeOffset) :
        key_mask.insert(kKeyTimeOffset);
        memcpy(&info.time_offset, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyTimeSyncType) :
        key_mask.insert(kKeyTimeSyncType);
        memcpy(&info.time_sync_type, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyStatusCode) :
        key_mask.insert(kKeyStatusCode);
        memcpy(&info.status_code, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLidarDiagStatus) :
        key_mask.insert(kKeyLidarDiagStatus);
        memcpy(&info.lidar_diag_status, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyLidarFlashStatus) :
        key_mask.insert(kKeyLidarFlashStatus);
        memcpy(&info.lidar_flash_status, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyFwType) :
        key_mask.insert(kKeyFwType);
        memcpy(&info.fw_type, &packet.data[offset], val_len);
        break; 
      case static_cast<uint16_t>(kKeyHmsCode) :
        key_mask.insert(kKeyHmsCode);
        memcpy(&info.hms_code, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeyRoiMode) :
        key_mask.insert(kKeyRoiMode);
        memcpy(&info.ROI_Mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetEscMode) :
        key_mask.insert(kKeySetEscMode);
        memcpy(&info.esc_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetFovMode) :
        key_mask.insert(kKeySetFovMode);
        memcpy(&info.fov_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetEchoMode) :
        key_mask.insert(kKeySetEchoMode);
        memcpy(&info.echo_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetNTPServerIp) : {
        key_mask.insert(kKeySetNTPServerIp);
        uint8_t ip[4];
        memcpy(ip, &packet.data[offset], sizeof(uint8_t) * 4);
        std::string ip_str = std::to_string(ip[0]) + "." + std::to_string(ip[1]) + "." +
            std::to_string(ip[2]) + "." + std::to_string(ip[3]);
        strcpy(info.ntp_server_ip.host_ip, ip_str.c_str());
        break;
      }
      case static_cast<uint16_t>(kKeySetITOCtrl) :
        key_mask.insert(kKeySetITOCtrl);
        memcpy(&info.ito_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetFogNoiseFilter) :
        key_mask.insert(kKeySetFogNoiseFilter);
        memcpy(&info.fog_noise_filter, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetTimeFilterMode) :
        key_mask.insert(kKeySetTimeFilterMode);
        memcpy(&info.time_filter_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetPclFreqMod) :
        key_mask.insert(kKeySetPclFreqMod);
        memcpy(&info.pcl_freq_mode, &packet.data[offset], val_len);
        break;
      case static_cast<uint16_t>(kKeySetImuRange) :
        key_mask.insert(kKeySetImuRange);
        memcpy(&info.imu_range, &packet.data[offset], val_len);
        break;
      default :
        break;
    }
    offset += val_len;
  }

  // printf("Lidar state info, pcl_data_type:%d, pattern_mode:%d, lidar_ip:%s, lidar_submask:%s, lidar_gatway:%s.\n",
  //     info.pcl_data_type, info.pattern_mode, info.lidar_ip_info.ip_addr, info.lidar_ip_info.net_mask, 
  //     info.lidar_ip_info.gw_addr);
  
  // printf("Lidar state info, host_ip_addr:%s, host_state_info_port:%u, lidar_state_info_port:%u.\n",
  //     info.host_state_info.host_ip_addr, info.host_state_info.host_state_info_port, info.host_state_info.lidar_state_info_port);

  // printf("Lidar state info, host_ip_addr:%s, host_point_data_port:%u, lidar_point_data_port:%u.\n",
  //     info.pointcloud_host_ipcfg.host_ip_addr, info.pointcloud_host_ipcfg.host_point_data_port,
  //     info.pointcloud_host_ipcfg.lidar_point_data_port);
    
  // printf("Lidar state info, host_ip_addr:%s, host_imu_data_port:%u, lidar_imu_data_port:%u.\n",
  //     info.imu_host_ipcfg.host_ip_addr, info.pointcloud_host_ipcfg.host_point_data_port,
  //     info.imu_host_ipcfg.lidar_imu_data_port);

  // printf("Lidar state info, roll:%f, pitch:%f, yaw:%f, x:%d, y:%d,z:%d.\n",
  //        info.install_attitude.roll_deg, info.install_attitude.pitch_deg,
  //        info.install_attitude.yaw_deg, info.install_attitude.x, info.install_attitude.y, info.install_attitude.z);
  
  // printf("Lidar state info, fov cfg0, yaw_start:%d, yaw_stop:%d, pitch_start:%d, pitch_stop:%d.\n",
  //     info.fov_cfg0.yaw_start, info.fov_cfg0.yaw_stop, info.fov_cfg0.pitch_start, info.fov_cfg0.pitch_stop);

  // printf("Lidar state info, fov cfg1, yaw_start:%d, yaw_stop:%d, pitch_start:%d, pitch_stop:%d.\n",
  //     info.fov_cfg1.yaw_start, info.fov_cfg1.yaw_stop, info.fov_cfg1.pitch_start, info.fov_cfg1.pitch_stop);

  // printf("Lidar state info, fov_en:%u, work_mode:%u, imu_data_en:%u, sn:%s, product_info:%s.\n",
  //     info.fov_en, info.work_mode, info.imu_data_en, info.sn, info.product_info);

  // std::string version_app = std::to_string(info.version_app[0]) + ":" + std::to_string(info.version_app[1]) + ":" + 
  //     std::to_string(info.version_app[2]) + ":" + std::to_string(info.version_app[3]);

  // std::string version_load = std::to_string(info.version_load[0]) + ":" + std::to_string(info.version_load[1]) + ":" + 
  //     std::to_string(info.version_load[2]) + ":" + std::to_string(info.version_load[3]);

  // std::string version_hardware = std::to_string(info.version_hardware[0]) + ":" + std::to_string(info.version_hardware[1]) + ":" + 
  //     std::to_string(info.version_hardware[2]) + ":" + std::to_string(info.version_hardware[3]);
  
  // std::string mac = std::to_string(info.mac[0]) + ":" + std::to_string(info.mac[1]) + ":" + 
  //     std::to_string(info.mac[2]) + ":" + std::to_string(info.mac[3]) + ":" +
  //     std::to_string(info.mac[4]) + ":" + std::to_string(info.mac[5]);

  // printf("Lidar state info, version_app:%s, version_load:%s, version_hardware:%s, mac:%s.\n",
  //     version_app.c_str(), version_load.c_str(), version_hardware.c_str(), mac.c_str());


  // printf("Lidar state info, cur_work_state:%u, core_temp:%d, powerup_cnt:%u, local_time_now:%lu, last_sync_time:%lu, time_offset:%ld.\n",
  //     info.cur_work_state, info.core_temp, info.powerup_cnt, info.local_time_now, info.last_sync_time, info.time_offset);
  
  // printf("Lidar state info, time_sync_type:%u, fw_type:%u.\n", info.time_sync_type, info.fw_type);

  return true;
}

void ParseLidarStateInfo::ParseLidarIpAddr(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t lidar_ip[4];
  memcpy(lidar_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_ip_str = std::to_string(lidar_ip[0]) + "." + std::to_string(lidar_ip[1]) + "." + 
      std::to_string(lidar_ip[2]) + "." + std::to_string(lidar_ip[3]);
  strcpy(info.lidar_ipcfg.ip_addr, lidar_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  uint8_t lidar_submask[4];
  memcpy(lidar_submask, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_submask_str = std::to_string(lidar_submask[0]) + "." + std::to_string(lidar_submask[1]) + 
      "." + std::to_string(lidar_submask[2]) + "." + std::to_string(lidar_submask[3]);
  strcpy(info.lidar_ipcfg.net_mask, lidar_submask_str.c_str());
  off += sizeof(uint8_t) * 4;
  
  uint8_t lidar_gateway[4];
  memcpy(lidar_gateway, &packet.data[off], sizeof(uint8_t) * 4);
  std::string lidar_gateway_str = std::to_string(lidar_gateway[0]) + "." + std::to_string(lidar_gateway[1]) +
      "." + std::to_string(lidar_gateway[2]) + "." + std::to_string(lidar_gateway[3]);
  strcpy(info.lidar_ipcfg.gw_addr, lidar_gateway_str.c_str());
}

void ParseLidarStateInfo::ParseStateInfoHostIPCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t host_state_info_ip[4];
  memcpy(host_state_info_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_state_info_ip_str = std::to_string(host_state_info_ip[0]) + "." + 
      std::to_string(host_state_info_ip[1]) + "." + std::to_string(host_state_info_ip[2]) + "." +
      std::to_string(host_state_info_ip[3]);
  
  strcpy(info.host_state_info.host_ip_addr, host_state_info_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.host_state_info.host_state_info_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.host_state_info.lidar_state_info_port, &packet.data[off], sizeof(uint16_t));
}

void ParseLidarStateInfo::ParsePointCloudHostIpCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t host_point_cloud_ip[4];
  memcpy(host_point_cloud_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_point_cloud_ip_str = std::to_string(host_point_cloud_ip[0]) + "." + 
      std::to_string(host_point_cloud_ip[1]) + "." + std::to_string(host_point_cloud_ip[2]) + "." +
      std::to_string(host_point_cloud_ip[3]);
  
  strcpy(info.pointcloud_host_ipcfg.host_ip_addr, host_point_cloud_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.pointcloud_host_ipcfg.host_point_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.pointcloud_host_ipcfg.lidar_point_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);
}

void ParseLidarStateInfo::ParseImuDataHostIpCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info) {
  uint8_t host_imu_data_ip[4];
  memcpy(host_imu_data_ip, &packet.data[off], sizeof(uint8_t) * 4);
  std::string host_imu_data_ip_str = std::to_string(host_imu_data_ip[0]) + "." + 
      std::to_string(host_imu_data_ip[1]) + "." + std::to_string(host_imu_data_ip[2]) + "." +
      std::to_string(host_imu_data_ip[3]);
  
  strcpy(info.imu_host_ipcfg.host_ip_addr, host_imu_data_ip_str.c_str());
  off += sizeof(uint8_t) * 4;

  memcpy(&info.imu_host_ipcfg.host_imu_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);

  memcpy(&info.imu_host_ipcfg.lidar_imu_data_port, &packet.data[off], sizeof(uint16_t));
  off += sizeof(uint16_t);
}

void ParseLidarStateInfo::ParseIpCfg(const CommPacket& packet, uint16_t off, LivoxIpCfg& cfg) {
  std::string ip_str = std::to_string(packet.data[off]) + "." + 
                       std::to_string(packet.data[off + 1]) + "." + 
                       std::to_string(packet.data[off + 2]) + "." +
                       std::to_string(packet.data[off + 3]);
  strcpy(cfg.ip_addr, ip_str.c_str());
  off += sizeof(uint8_t) * 4;
  cfg.dst_port = *(uint16_t*)&packet.data[off];
  off += sizeof(uint16_t); 
  cfg.src_port = *(uint16_t*)&packet.data[off];
  return;
}

namespace {

// Fixed-size char fields from the lidar are not guaranteed to be NUL-terminated.
template <size_t N>
std::string FieldString(const char (&field)[N]) {
  return std::string(field, strnlen(field, N));
}

template <typename T, size_t N>
nlohmann::ordered_json FieldArray(const T (&field)[N]) {
  nlohmann::ordered_json arr = nlohmann::ordered_json::array();
  for (size_t i = 0; i < N; ++i) {
    arr.push_back(field[i]);
  }
  return arr;
}

template <typename T>
nlohmann::ordered_json IpCfg(const char (&ip)[16], T dst_port, T src_port) {
  return {{"ip", FieldString(ip)}, {"dst_port", dst_port}, {"src_port", src_port}};
}

nlohmann::ordered_json Fov(const FovCfg& fov) {
  return {{"yaw_start", fov.yaw_start}, {"yaw_stop", fov.yaw_stop},
          {"pitch_start", fov.pitch_start}, {"pitch_stop", fov.pitch_stop}};
}

} // namespace

void ParseLidarStateInfo::LivoxLidarStateInfoToJson(const DirectLidarStateInfo& info, const std::set<ParamKeyName>& key_mask, std::string& lidar_info) {
  // ordered_json keeps keys in insertion order, matching the previous output.
  nlohmann::ordered_json j = nlohmann::ordered_json::object();
  auto has = [&key_mask](ParamKeyName key) { return key_mask.find(key) != key_mask.end(); };

  if (has(kKeyPclDataType)) j["pcl_data_type"] = info.pcl_data_type;
  if (has(kKeyPatternMode)) j["pattern_mode"] = info.pattern_mode;
  if (has(kKeyDualEmitEn)) j["dual_emit_en"] = info.dual_emit_en;
  if (has(kKeyPointSendEn)) j["point_send_en"] = info.point_send_en;
  if (has(kKeyLidarIpCfg)) {
    j["lidar_ipcfg"] = {{"lidar_ip", FieldString(info.lidar_ipcfg.ip_addr)},
                        {"lidar_subnet_mask", FieldString(info.lidar_ipcfg.net_mask)},
                        {"lidar_gateway", FieldString(info.lidar_ipcfg.gw_addr)}};
  }
  if (has(kKeyStateInfoHostIpCfg)) {
    j["state_info_host_ipcfg"] = IpCfg(info.host_state_info.host_ip_addr,
        info.host_state_info.host_state_info_port, info.host_state_info.lidar_state_info_port);
  }
  if (has(kKeyLidarPointDataHostIpCfg)) {
    j["ponitcloud_host_ipcfg"] = IpCfg(info.pointcloud_host_ipcfg.host_ip_addr,
        info.pointcloud_host_ipcfg.host_point_data_port, info.pointcloud_host_ipcfg.lidar_point_data_port);
  }
  if (has(kKeyLidarImuHostIpCfg)) {
    j["imu_host_ipcfg"] = IpCfg(info.imu_host_ipcfg.host_ip_addr,
        info.imu_host_ipcfg.host_imu_data_port, info.imu_host_ipcfg.lidar_imu_data_port);
  }
  if (has(kKeyCtlHostIpCfg)) {
    j["ctl_host_ipcfg"] = IpCfg(info.ctl_host_ipcfg.ip_addr, info.ctl_host_ipcfg.dst_port, info.ctl_host_ipcfg.src_port);
  }
  if (has(kKeyLogHostIpCfg)) {
    j["log_host_ipcfg"] = IpCfg(info.log_host_ipcfg.ip_addr, info.log_host_ipcfg.dst_port, info.log_host_ipcfg.src_port);
  }
  if (has(kKeyVehicleSpeed)) j["vehicle_speed"] = info.vehicle_speed;
  if (has(kKeyEnvironmentTemp)) j["environment_temp"] = info.environment_temp;
  if (has(kKeyInstallAttitude)) {
    const LivoxLidarInstallAttitude& att = info.install_attitude;
    j["install_attitude"] = {{"roll_deg", static_cast<double>(att.roll_deg)},
                             {"pitch_deg", static_cast<double>(att.pitch_deg)},
                             {"yaw_deg", static_cast<double>(att.yaw_deg)},
                             {"x_mm", att.x}, {"y_mm", att.y}, {"z_mm", att.z}};
  }
  if (has(kKeyBlindSpotSet)) j["blind_spot_set"] = info.blind_spot_set;
  if (has(kKeyFrameRate)) j["frame_rate"] = info.frame_rate;
  if (has(kKeyFovCfg0)) j["fov_cfg0"] = Fov(info.fov_cfg0);
  if (has(kKeyFovCfg1)) j["fov_cfg1"] = Fov(info.fov_cfg1);
  if (has(kKeyFovCfgEn)) j["fov_cfg_en"] = info.fov_cfg_en;
  if (has(kKeyDetectMode)) j["detect_mode"] = info.detect_mode;
  if (has(kKeyFuncIoCfg)) {
    j["func_io_cfg"] = {{"IN0", info.func_io_cfg[0]}, {"IN1", info.func_io_cfg[1]},
                        {"OUT0", info.func_io_cfg[2]}, {"OUT1", info.func_io_cfg[3]}};
  }
  if (has(kKeyWorkMode)) j["work_tgt_mode"] = info.work_tgt_mode;
  if (has(kKeyGlassHeat)) j["glass_heat"] = info.glass_heat;
  if (has(kKeyImuDataEn)) j["imu_data_en"] = info.imu_data_en;
  if (has(kKeyFusaEn)) j["fusa_en"] = info.fusa_en;
  if (has(kKeySetEscMode)) j["esc_mode"] = info.esc_mode;
  if (has(kKeySetFovMode)) j["fov_mode"] = info.fov_mode;
  if (has(kKeySetEchoMode)) j["echo_mode"] = info.echo_mode;
  if (has(kKeySetNTPServerIp)) j["ntp_server_ip"] = FieldString(info.ntp_server_ip.host_ip);
  if (has(kKeySetITOCtrl)) j["ito_mode"] = info.ito_mode;
  if (has(kKeySetFogNoiseFilter)) j["fog_noise_filter"] = info.fog_noise_filter;
  if (has(kKeySetPclFreqMod)) j["pcl_freq_mode"] = info.pcl_freq_mode;
  if (has(kKeySetTimeFilterMode)) j["time_filter_mode"] = info.time_filter_mode;
  if (has(kKeySetImuRange)) {
    j["imu_range"] = {{"imu_out_rate", info.imu_range.imu_out_rate},
                      {"accel_range", info.imu_range.accel_range},
                      {"gyro_range", info.imu_range.gyro_range}};
  }
  if (has(kKeySn)) j["sn"] = FieldString(info.sn);
  if (has(kKeyProductInfo)) j["product_info"] = FieldString(info.product_info);
  if (has(kKeyVersionApp)) j["version_app"] = FieldArray(info.version_app);
  if (has(kKeyVersionLoader)) j["version_loader"] = FieldArray(info.version_loader);
  if (has(kKeyVersionHardware)) j["version_hardware"] = FieldArray(info.version_hardware);
  if (has(kKeyMac)) j["mac"] = FieldArray(info.mac);
  if (has(kKeyCurWorkState)) j["cur_work_state"] = info.cur_work_state;
  if (has(kKeyCoreTemp)) j["core_temp"] = info.core_temp;
  if (has(kKeyPowerUpCnt)) j["powerup_cnt"] = info.powerup_cnt;
  if (has(kKeyLocalTimeNow)) j["local_time_now"] = info.local_time_now;
  if (has(kKeyLastSyncTime)) j["last_sync_time"] = info.last_sync_time;
  if (has(kKeyTimeOffset)) j["time_offset"] = info.time_offset;
  if (has(kKeyTimeSyncType)) j["time_sync_type"] = info.time_sync_type;
  if (has(kKeyStatusCode)) {
    std::ostringstream ss;
    for (int idx = 31; idx >= 0; --idx) {
      ss << std::hex << static_cast<uint32_t>(info.status_code[idx]);
      if (idx != 0) {
        ss << " ";
      }
    }
    j["status_code"] = ss.str();
  }
  if (has(kKeyLidarDiagStatus)) j["lidar_diag_status"] = info.lidar_diag_status;
  if (has(kKeyLidarFlashStatus)) j["lidar_flash_status"] = info.lidar_flash_status;
  if (has(kKeyFwType)) j["FW_TYPE"] = info.fw_type;
  if (has(kKeyHmsCode)) j["hms_code"] = FieldArray(info.hms_code);
  if (has(kKeyRoiMode)) j["ROI_Mode"] = info.ROI_Mode;

  // Replace invalid UTF-8 from device strings instead of throwing on the IO thread.
  lidar_info = j.dump(4, ' ', false, nlohmann::ordered_json::error_handler_t::replace);
}

} // namespace livox
} // namespace direct





