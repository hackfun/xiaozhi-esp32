#include "radar/radar_tools.h"

#include <cstdlib>
#include <string>

#include <cJSON.h>
#include <esp_log.h>

#include "mcp_server.h"
#include "radar/radar_service.h"

#define TAG "RadarTools"

namespace radar {
namespace {

std::string StatusToJson(const RadarStatus& status) {
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr) {
        return "{}";
    }

    cJSON* gps = cJSON_AddObjectToObject(root, "gps");
    cJSON_AddBoolToObject(gps, "valid", status.gps_valid);
    cJSON_AddNumberToObject(gps, "satellites", status.satellites);
    cJSON_AddNumberToObject(gps, "hdop", status.hdop);
    cJSON_AddNumberToObject(gps, "lat", status.latitude);
    cJSON_AddNumberToObject(gps, "lon", status.longitude);

    cJSON* target = cJSON_AddObjectToObject(root, "target");
    cJSON_AddBoolToObject(target, "locked", status.has_target);
    if (status.has_target) {
        cJSON_AddStringToObject(target, "id", status.target_id.c_str());
        cJSON_AddStringToObject(target, "name", status.target_name.c_str());
        cJSON_AddNumberToObject(target, "distance_m", status.distance_m);
        cJSON_AddNumberToObject(target, "bearing_deg", status.bearing_deg);
    }

    cJSON_AddStringToObject(root, "phase", status.phase);

    char* text = cJSON_PrintUnformatted(root);
    std::string result = (text != nullptr) ? std::string(text) : std::string("{}");
    if (text != nullptr) {
        cJSON_free(text);
    }
    cJSON_Delete(root);
    return result;
}

}  // namespace

void RegisterRadarTools() {
    auto& server = McpServer::GetInstance();

    server.AddTool(
        "self.radar.get_status",
        "获取身上雷达的实时状态：定位是否有效、卫星数量、精度、当前锁定的宝箱名称、"
        "距离和方位。当用户问「还有多远」「宝箱在哪」「信号怎么样」时调用。\n"
        "distance_m 单位是米，bearing_deg 是相对真北的方位角（0=正北，90=正东）。",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            return StatusToJson(RadarService::GetInstance().GetStatus());
        });

    // 调试用：没有 GPS 硬件时手动灌一个坐标，把「感知 -> 播报」链路先跑通。
    // 正式出货版本应当移除这个工具，否则等于给用户开了个伪装定位的后门。
    server.AddTool(
        "self.radar.debug_inject_position",
        "【仅调试】手动注入一个 GPS 坐标，绕过真实定位硬件。"
        "只有在明确要求「模拟位置」时才调用，正常对话绝对不要调用。",
        PropertyList({Property("lat", kPropertyTypeString), Property("lon", kPropertyTypeString)}),
        [](const PropertyList& properties) -> ReturnValue {
            const std::string lat_text = properties["lat"].value<std::string>();
            const std::string lon_text = properties["lon"].value<std::string>();
            const double latitude = std::atof(lat_text.c_str());
            const double longitude = std::atof(lon_text.c_str());
            if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
                return std::string("坐标超出范围");
            }
            RadarService::GetInstance().InjectFix(latitude, longitude);
            ESP_LOGW(TAG, "debug inject (%.6f, %.6f)", latitude, longitude);
            return std::string("已注入坐标");
        });

    ESP_LOGI(TAG, "radar tools registered");
}

}  // namespace radar
