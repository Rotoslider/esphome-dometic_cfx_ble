#ifndef DOMETIC_CFX_BLE_H
#define DOMETIC_CFX_BLE_H

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/entity_base.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gatt_defs.h>

#include <vector>
#include <queue>
#include <map>
#include <string>

namespace esphome {
namespace dometic_cfx_ble {

static const char *const TAG = "dometic_cfx_ble";

struct TopicInfo {
  std::string name;
  uint8_t param[4];
  std::string type;
  std::string desc;
};

const std::map<std::string, TopicInfo> TOPICS = {
    {"SUBSCRIBE_APP_SZ", {"SUBSCRIBE_APP_SZ", {1,0,0,129}, "EMPTY", "Subscribe all SZ"}},
    // Add all others as in Python
    // ... (copy all entries from Python TOPICS dict, convert to C++)
    {"DC_CURRENT_HISTORY_WEEK", {"DC_CURRENT_HISTORY_WEEK", {0,66,3,1}, "HISTORY_DATA_ARRAY", "DC current week history"}},
};

const std::map<uint8_t, std::string> ACTION_NAMES = {
    {0, "PUB"},
    {1, "SUB"},
    {2, "PING"},
    {3, "HELLO"},
    {4, "ACK"},
    {5, "NAK"},
    {6, "NOP"},
};

const std::map<uint8_t, std::string> BATTERY_LEVELS = {{0, "Low"}, {1, "Medium"}, {2, "High"}};
const std::map<uint8_t, std::string> POWER_SOURCES = {{0, "AC"}, {1, "DC"}, {2, "Solar"}};
const float NO_VALUE = -3276.8f;

class DometicCfxBle : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  void set_mac_address(const uint8_t *mac) { memcpy(mac_address_, mac, 6); }
  void set_product_type(uint8_t type) { product_type_ = type; }

  void add_entity(const std::string &topic, EntityBase *entity) { entities_[topic] = entity; }

  void setup() override;
  void loop() override;

  void send_pub(const std::string &topic, const std::vector<uint8_t> &value);
  void send_sub(const std::string &topic);
  void send_ping();

 protected:
  uint8_t mac_address_[6];
  uint8_t product_type_;
  esp_gatt_if_t gattc_if_;
  uint16_t gattc_conn_id_;
  bool connected_;
  uint16_t write_handle_;
  uint16_t notify_handle_;
  std::queue<std::vector<uint8_t>> send_queue_;
  std::map<std::string, EntityBase *> entities_;
  uint32_t last_activity_time_;
  bool should_rescan_;

  static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gattc_cb_param_t *param);

  void connect();
  void reconnect();
  void start_scan();
  void handle_notify(const uint8_t *data, size_t len);
  void update_entity(const std::string &topic, const std::vector<uint8_t> &value);
  std::vector<uint8_t> encode_value(const std::any &value, const std::string &type);
  std::any decode_value(const std::vector<uint8_t> &bytes, const std::string &type);
  std::string get_english_desc(const std::string &topic, const std::any &value);
};

}  // namespace dometic_cfx_ble
}  // namespace esphome

#endif