#ifndef AMR_VISUALIZATION__AI_CHAT_PANEL_HPP_
#define AMR_VISUALIZATION__AI_CHAT_PANEL_HPP_

/**
 * @file ai_chat_panel.hpp
 * @brief Qt reconstruction of the RCS AI mission control panel.
 */

#include <QWidget>
#include <QString>

class QLabel;
class QPushButton;
class QScrollArea;
class QTextEdit;
class QVBoxLayout;

namespace amr::visualization
{

/// @brief AI mission chat panel backed by the ROS-native AMR MCP chat service.
class AiChatPanel : public QWidget
{
  Q_OBJECT

public:
  /// @brief Construct the panel and initialize offline transcript state.
  explicit AiChatPanel(QWidget *parent = nullptr);

Q_SIGNALS:
  /// @brief Request publishing the current scene aim pose as the initial pose.
  void initialPoseRequested();
  /// @brief Request sending the current scene target or route.
  void goalRequested();
  /// @brief Request appending the current scene aim pose as a waypoint.
  void waypointRequested();
  /// @brief Request canceling navigation.
  void cancelRequested();
  /// @brief Request clearing scene waypoints and target state.
  void clearRequested();
  /// @brief Request an AI chat service call through the ROS worker.
  void chatRequested(
    const QString &provider,
    const QString &robot_id,
    const QString &default_frame,
    const QString &message);
  /// @brief Request the ROS worker to use a different MCP chat service name.
  void chatServiceNameChanged(const QString &service_name);

private Q_SLOTS:
  /// @brief Open ROS service settings for the MCP chat backend.
  void openSettingsDialog();
  /// @brief Send the current input text as a ROS service request.
  void sendMessage();
  /// @brief Remove transcript messages and re-add the system intro.
  void clearTranscript();

public Q_SLOTS:
  /// @brief Display the ROS service response from the MCP server.
  void handleChatResponse(
    bool accepted,
    bool command_executed,
    const QString &command_type,
    const QString &response,
    const QString &request_id,
    const QString &error_message);
  /// @brief Display one asynchronous MCP feedback event.
  void handleMcpFeedback(const QString &message);
  /// @brief Display service availability state.
  void setServiceAvailable(bool available);

private:
  /// @brief Add one role-styled message bubble to the transcript.
  void addTranscriptMessage(const QString &role, const QString &message);
  /// @brief Place a recommended prompt in the input and focus it.
  void applyPrompt(const QString &prompt);
  /// @brief Refresh summary text and online/offline pill.
  void updateConnectionUi();
  /// @brief Return the JSON provider id for the active provider.
  QString providerId() const;

  QLabel *summary_label_{nullptr};
  QLabel *status_pill_{nullptr};
  QScrollArea *transcript_scroll_{nullptr};
  QWidget *transcript_content_{nullptr};
  QVBoxLayout *transcript_layout_{nullptr};
  QTextEdit *input_{nullptr};
  QPushButton *send_button_{nullptr};
  QString service_name_{"/amr_mcp/chat"};
  QString provider_label_{"Ollama"};
  QString robot_id_{"burger1"};
  QString default_frame_{"map"};
  bool service_available_{false};
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__AI_CHAT_PANEL_HPP_
