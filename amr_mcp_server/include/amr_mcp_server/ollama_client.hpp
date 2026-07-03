#ifndef AMR_MCP_SERVER__OLLAMA_CLIENT_HPP_
#define AMR_MCP_SERVER__OLLAMA_CLIENT_HPP_

#include <string>

#include "amr_mcp_server/topic_snapshot.hpp"

namespace amr::mcp
{

struct OllamaConfig
{
  std::string base_url{"http://127.0.0.1:11434"};
  std::string chat_endpoint{"/api/chat"};
  std::string model{"qwen3"};
  int timeout_ms{30000};
  bool enabled{true};
};

class OllamaClient
{
public:
  explicit OllamaClient(OllamaConfig config);

  std::string ask(
    const std::string &message,
    const SnapshotData &snapshot,
    std::string *error_message) const;

private:
  static std::string escapeJson(const std::string &text);
  static std::string extractAssistantContent(const std::string &payload);
  static size_t writeCallback(char *ptr, size_t size, size_t nmemb, void *userdata);

  OllamaConfig config_;
};

}  // namespace amr::mcp

#endif  // AMR_MCP_SERVER__OLLAMA_CLIENT_HPP_
