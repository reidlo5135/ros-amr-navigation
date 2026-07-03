#include "amr_mcp_server/ollama_client.hpp"

#include <curl/curl.h>

#include <sstream>
#include <utility>

namespace amr::mcp
{

OllamaClient::OllamaClient(OllamaConfig config)
: config_(std::move(config))
{
}

std::string OllamaClient::ask(
  const std::string &message,
  const SnapshotData &snapshot,
  std::string *error_message) const
{
  if (!config_.enabled) {
    if (error_message) {
      *error_message = "Ollama is disabled by parameter.";
    }
    return {};
  }

  CURL *curl = curl_easy_init();
  if (!curl) {
    if (error_message) {
      *error_message = "Failed to initialize libcurl.";
    }
    return {};
  }

  std::ostringstream snapshot_text;
  snapshot_text << "battery=" << (snapshot.has_battery ? std::to_string(snapshot.battery_percentage) : "none");
  snapshot_text << ", pose=";
  if (snapshot.has_pose) {
    snapshot_text << "x " << snapshot.pose_x << " y " << snapshot.pose_y << " yaw " << snapshot.pose_yaw;
  } else {
    snapshot_text << "none";
  }
  snapshot_text << ", motion_active=" << (snapshot.motion_active ? "true" : "false");
  snapshot_text << ", blocked=" << (snapshot.motion_blocked ? "true" : "false");
  snapshot_text << ", runtime_state=" << snapshot.runtime_state;

  const std::string system_prompt =
    "너는 ROS2 Humble 기반 AMR 운영 보조 AI다. 항상 한국어로 간결하게 답한다. "
    "로봇 제어 명령은 허용된 command schema 안에서만 판단한다. 좌표는 기본적으로 map frame 기준이다. "
    "확실하지 않은 좌표나 위험한 직접 속도 명령은 실행하지 않고 확인을 요청한다. "
    "사용자가 상태 요약을 요청하면 제공된 topic snapshot만 기반으로 답한다. "
    "존재하지 않는 센서값이나 topic 상태를 지어내지 않는다.";

  const std::string body =
    "{\"model\":\"" + escapeJson(config_.model) + "\",\"stream\":false,\"messages\":["
    "{\"role\":\"system\",\"content\":\"" + escapeJson(system_prompt) + "\"},"
    "{\"role\":\"user\",\"content\":\"topic snapshot: " + escapeJson(snapshot_text.str()) +
    "\\noperator request: " + escapeJson(message) + "\"}]}";

  std::string response_body;
  const std::string url = config_.base_url + config_.chat_endpoint;
  struct curl_slist *headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(config_.timeout_ms));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &OllamaClient::writeCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);

  const CURLcode result = curl_easy_perform(curl);
  long response_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (result != CURLE_OK) {
    if (error_message) {
      *error_message = curl_easy_strerror(result);
    }
    return {};
  }
  if (response_code < 200 || response_code >= 300) {
    if (error_message) {
      *error_message = "Ollama HTTP status " + std::to_string(response_code);
    }
    return {};
  }

  std::string content = extractAssistantContent(response_body);
  if (content.empty()) {
    content = response_body;
  }
  return content;
}

std::string OllamaClient::escapeJson(const std::string &text)
{
  std::string escaped;
  escaped.reserve(text.size());
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += ch;
        break;
    }
  }
  return escaped;
}

std::string OllamaClient::extractAssistantContent(const std::string &payload)
{
  const std::string key = "\"content\"";
  const auto key_pos = payload.find(key);
  if (key_pos == std::string::npos) {
    return {};
  }
  const auto colon_pos = payload.find(':', key_pos + key.size());
  const auto first_quote = payload.find('"', colon_pos + 1U);
  if (colon_pos == std::string::npos || first_quote == std::string::npos) {
    return {};
  }

  std::string content;
  bool escaped = false;
  for (size_t i = first_quote + 1U; i < payload.size(); ++i) {
    const char ch = payload[i];
    if (escaped) {
      if (ch == 'n') {
        content += '\n';
      } else if (ch == 't') {
        content += '\t';
      } else {
        content += ch;
      }
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      break;
    }
    content += ch;
  }
  return content;
}

size_t OllamaClient::writeCallback(char *ptr, size_t size, size_t nmemb, void *userdata)
{
  auto *response = static_cast<std::string *>(userdata);
  const size_t bytes = size * nmemb;
  response->append(ptr, bytes);
  return bytes;
}

}  // namespace amr::mcp
