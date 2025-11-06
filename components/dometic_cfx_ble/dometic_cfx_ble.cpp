#include "dometic_cfx_ble.h"

namespace esphome {
namespace dometic_cfx_ble {

void DometicCfxBle::setup() {
  ESP_LOGI(TAG, "Setting up Dometic CFX BLE...");
  last_activity_time_ = millis();
  should_rescan_ = false;
  connected_ = false;

  esp_err_t ret = esp_ble_gattc_app_register(0);
  if (ret) {
    ESP_LOGE(TAG, "gattc app register failed: %d", ret);
  }

  esp_ble_gap_register_callback(gap_event_handler);
  start_scan();
}

void DometicCfxBle::loop() {
  uint32_t now = millis();
  if (connected_) {
    if (now - last_activity_time_ > 3000) {
      send_ping();
      last_activity_time_ = now;
    }
    if (!send_queue_.empty()) {
      auto packet = send_queue_.front();
      esp_err_t ret = esp_ble_gattc_write_char(this->gattc_if_, this->gattc_conn_id_, this->write_handle_, packet.size(), packet.data(), ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      if (ret == ESP_OK) {
        // Wait for ACK in event, then pop
      } else {
        ESP_LOGE(TAG, "Write failed: %d", ret);
        send_queue_.pop();  // Or retry
      }
    }
  } else if (should_rescan_) {
    start_scan();
    should_rescan_ = false;
  } else if (now - last_activity_time_ > 5000) {
    reconnect();
  }
}

void DometicCfxBle::send_pub(const std::string &topic, const std::vector<uint8_t> &value) {
  auto it = TOPICS.find(topic);
  if (it != TOPICS.end()) {
    std::vector<uint8_t> packet = {ACTION_PUB};
    packet.insert(packet.end(), it->second.param, it->second.param + 4);
    packet.insert(packet.end(), value.begin(), value.end());
    send_queue_.push(packet);
  }
}

void DometicCfxBle::send_sub(const std::string &topic) {
  auto it = TOPICS.find(topic);
  if (it != TOPICS.end()) {
    std::vector<uint8_t> packet = {ACTION_SUB};
    packet.insert(packet.end(), it->second.param, it->second.param + 4);
    send_queue_.push(packet);
  }
}

void DometicCfxBle::send_ping() {
  std::vector<uint8_t> packet = {ACTION_PING};
  send_queue_.push(packet);
}

void DometicCfxBle::start_scan() {
  esp_ble_scan_params_t scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE
  };
  esp_ble_gap_set_scan_params(&scan_params);
  esp_ble_gap_start_scanning(10);  // 10s
}

void DometicCfxBle::connect() {
  esp_ble_gattc_open(this->gattc_if_, this->mac_address_, BLE_ADDR_TYPE_PUBLIC, true);
}

void DometicCfxBle::reconnect() {
  connected_ = false;
  should_rescan_ = true;
}

void DometicCfxBle::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      esp_ble_gap_start_scanning(10);
      break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
      if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
        // Check MAC
        if (memcmp(param->scan_rst.bda, mac_address_, 6) == 0) {
          esp_ble_gap_stop_scanning();
          connect();
        }
      }
      break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
      break;
    default:
      break;
  }
}

void DometicCfxBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_REG_EVT:
      gattc_if_ = param->reg.app_id;
      break;
    case ESP_GATTC_OPEN_EVT:
      if (param->open.status == ESP_GATT_OK) {
        conn_id_ = param->open.conn_id;
        esp_ble_gattc_search_service(gattc_if_, conn_id_, nullptr);
      }
      break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
      // Get char handles
      esp_ble_gattc_get_char_by_uuid(gattc_if_, conn_id_, 0, 0xffff, remote_write_char_uuid, &write_handle, nullptr);
      esp_ble_gattc_get_char_by_uuid(gattc_if_, conn_id_, 0, 0xffff, remote_notify_char_uuid, &notify_handle, nullptr);
      // Enable notify
      uint8_t cccd[2] = {0x01, 0x00};
      esp_ble_gattc_write_char_desc(gattc_if_, conn_id_, notify_handle + 1, 2, cccd, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      connected_ = true;
      // Send ping
      send_ping();
      // Sub group
      std::string sub_key = (product_type_ == 1) ? "SUBSCRIBE_APP_SZ" : (product_type_ == 2) ? "SUBSCRIBE_APP_SZI" : "SUBSCRIBE_APP_DZ";
      send_sub(sub_key);
      break;
    case ESP_GATTC_NOTIFY_EVT:
      handle_notify(param->notify.value, param->notify.value_len);
      last_activity_time_ = millis();
      break;
    case ESP_GATTC_DISCONNECT_EVT:
      connected_ = false;
      break;
    // Handle write rsp for ACK
    case ESP_GATTC_WRITE_CHAR_EVT:
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "Write error");
      }
      break;
    default:
      break;
  }
}

void DometicCfxBle::handle_notify(const uint8_t *data, size_t len) {
  if (len < 1) return;
  uint8_t action = data[0];
  if (action == ACTION_ACK || action == ACTION_NAK) {
    if (!send_queue_.empty()) {
      send_queue_.pop();
    }
  }
  if (action == ACTION_PUB && len >= 5) {
    uint8_t topic_param[4];
    memcpy(topic_param, data + 1, 4);
    std::string topic;
    for (const auto& pair : TOPICS) {
      if (memcmp(pair.second.param, topic_param, 4) == 0) {
        topic = pair.first;
        break;
      }
    }
    if (!topic.empty()) {
      std::vector<uint8_t> value(data + 5, data + len);
      update_entity(topic, value);
    }
  }
  // Send ACK
  uint8_t ack = ACTION_ACK;
  esp_ble_gattc_write_char(gattc_if_, conn_id_, write_handle_, 1, &ack, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
}

void DometicCfxBle::update_entity(const std::string &topic, const std::vector<uint8_t> &value) {
  auto it = entities_.find(topic);
  if (it != entities_.end()) {
    auto type = TOPICS.at(topic).type;
    auto decoded = decode_value(value, type);
    if (auto sensor = dynamic_cast<sensor::Sensor *>(it->second)) {
      sensor->publish_state(std::get<float>(decoded));
    } else if (auto bin_sensor = dynamic_cast<binary_sensor::BinarySensor *>(it->second)) {
      bin_sensor->publish_state(std::get<bool>(decoded));
    } else if (auto sw = dynamic_cast<switch_::Switch *>(it->second)) {
      sw->publish_state(std::get<bool>(decoded));
    } else if (auto num = dynamic_cast<number::Number *>(it->second)) {
      num->publish_state(std::get<float>(decoded));
    } else if (auto text = dynamic_cast<text_sensor::TextSensor *>(it->second)) {
      text->publish_state(std::get<std::string>(decoded));
    }
  }
}

// Implement encode_value and decode_value in C++ using memcpy/unpack logic for types
std::vector<uint8_t> DometicCfxBle::encode_value(const std::any &value, const std::string &type) {
  std::vector<uint8_t> bytes;
  // Switch on type, cast any to proper, pack
  return bytes;
}

std::any DometicCfxBle::decode_value(const std::vector<uint8_t> &bytes, const std::string &type) {
  // Switch on type, unpack to float/bool/string/etc
  return std::any();
}

std::string DometicCfxBle::get_english_desc(const std::string &topic, const std::any &value) {
  // Impl as in Python
  return "";
}

}  // namespace dometic_cfx_ble
}  // namespace esphome