
#include <string>
#include <nlohmann/json.hpp>

int viv_get_ctrl(int fd, const char* config_id);
int viv_set_ctrl(int fd, const char* config_id, const std::string& key, const nlohmann::json& value);