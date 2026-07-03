/**
 * @file ai_chat_panel.cpp
 * @brief Implementation of the AI mission control panel.
 */

#include "amr_visualization/ai_chat_panel.hpp"

#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QLayoutItem>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSizePolicy>
#include <QStringList>
#include <QStyle>
#include <QTextEdit>
#include <QTextCursor>
#include <QTimer>
#include <QVBoxLayout>

namespace amr::visualization
{

namespace
{

/// @brief Recommended prompts mirrored from the target RCS AI panel.
QStringList recommended_prompts()
{
  return {
    QString::fromUtf8(
      "로봇을 map 기준 x=0.00, y=0.00으로 보내줘."),
    QString::fromUtf8(
      "주행을 취소해줘.")
  };
}

}  // namespace

/// @copydoc AiChatPanel::AiChatPanel
AiChatPanel::AiChatPanel(QWidget *parent)
: QWidget(parent)
{
  setObjectName("aiMissionPanel");
  setMinimumWidth(520);
  setMaximumWidth(860);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(14, 14, 16, 14);
  layout->setSpacing(11);

  auto *header = new QWidget;
  auto *header_layout = new QHBoxLayout(header);
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(6);

  auto *title = new QLabel("AI MISSION CONTROL");
  title->setObjectName("aiPanelTitle");
  status_pill_ = new QLabel("OFFLINE");
  status_pill_->setObjectName("aiStatusPill");
  auto *clear_button = new QPushButton("Clear");
  clear_button->setObjectName("aiClearButton");
  auto *settings_button = new QPushButton(QString::fromUtf8("⚙"));
  settings_button->setObjectName("aiSettingsButton");
  settings_button->setToolTip("AI connection settings");

  header_layout->addWidget(title);
  header_layout->addStretch(1);
  header_layout->addWidget(status_pill_);
  header_layout->addWidget(clear_button);
  header_layout->addWidget(settings_button);
  layout->addWidget(header);

  summary_label_ = new QLabel;
  summary_label_->setObjectName("aiSummaryLabel");
  summary_label_->setWordWrap(true);
  layout->addWidget(summary_label_);

  auto *tool_row = new QWidget;
  auto *tool_layout = new QHBoxLayout(tool_row);
  tool_layout->setContentsMargins(0, 0, 0, 0);
  tool_layout->setSpacing(8);
  auto *tool_label = new QLabel(QString::fromUtf8("맵에서 좌표 가져오기"));
  tool_label->setObjectName("aiMutedLabel");
  auto *initial_pose_button = new QPushButton(QString::fromUtf8("초기 위치"));
  auto *goal_button = new QPushButton(QString::fromUtf8("목표"));
  auto *waypoint_button = new QPushButton(QString::fromUtf8("경유"));
  auto *cancel_button = new QPushButton(QString::fromUtf8("취소"));
  auto *clear_map_button = new QPushButton(QString::fromUtf8("지우기"));
  tool_layout->addWidget(tool_label);
  tool_layout->addStretch(1);
  for (auto *button : {initial_pose_button, goal_button, waypoint_button, cancel_button, clear_map_button}) {
    button->setObjectName("aiToolButton");
    tool_layout->addWidget(button);
  }
  cancel_button->setObjectName("aiDangerToolButton");
  layout->addWidget(tool_row);

  auto *prompt_title = new QLabel(QString::fromUtf8("추천 AI Prompt"));
  prompt_title->setObjectName("aiSubsectionTitle");
  layout->addWidget(prompt_title);

  const QStringList prompts = recommended_prompts();
  for (int i = 0; i < prompts.size(); ++i) {
    auto *prompt_button = new QPushButton(prompts[i]);
    prompt_button->setObjectName("promptButton");
    prompt_button->setToolTip(prompts[i]);
    connect(prompt_button, &QPushButton::clicked, this, [this, prompt = prompts[i]]() {
      applyPrompt(prompt);
    });
    layout->addWidget(prompt_button);
  }

  transcript_scroll_ = new QScrollArea;
  transcript_scroll_->setObjectName("transcriptScroll");
  transcript_scroll_->setWidgetResizable(true);
  transcript_content_ = new QWidget;
  transcript_content_->setObjectName("transcriptContent");
  transcript_layout_ = new QVBoxLayout(transcript_content_);
  transcript_layout_->setContentsMargins(8, 8, 8, 8);
  transcript_layout_->setSpacing(7);
  transcript_layout_->setAlignment(Qt::AlignTop);
  transcript_scroll_->setWidget(transcript_content_);
  layout->addWidget(transcript_scroll_, 1);

  input_ = new QTextEdit;
  input_->setObjectName("aiInput");
  input_->setAcceptRichText(false);
  input_->setPlaceholderText(QString::fromUtf8("활성 로봇에 대해 Ollama에게 물어보거나, 실행할 작업을 적어보세요."));
  input_->setMinimumHeight(92);
  input_->setMaximumHeight(112);
  layout->addWidget(input_);

  send_button_ = new QPushButton;
  send_button_->setObjectName("aiSendButton");
  layout->addWidget(send_button_);

  connect(clear_button, &QPushButton::clicked, this, &AiChatPanel::clearTranscript);
  connect(settings_button, &QPushButton::clicked, this, &AiChatPanel::openSettingsDialog);
  connect(send_button_, &QPushButton::clicked, this, &AiChatPanel::sendMessage);
  connect(initial_pose_button, &QPushButton::clicked, this, &AiChatPanel::initialPoseRequested);
  connect(goal_button, &QPushButton::clicked, this, &AiChatPanel::goalRequested);
  connect(waypoint_button, &QPushButton::clicked, this, &AiChatPanel::waypointRequested);
  connect(cancel_button, &QPushButton::clicked, this, &AiChatPanel::cancelRequested);
  connect(clear_map_button, &QPushButton::clicked, this, &AiChatPanel::clearRequested);

  updateConnectionUi();
  clearTranscript();
}

/// @copydoc AiChatPanel::openSettingsDialog
void AiChatPanel::openSettingsDialog()
{
  QDialog dialog(this);
  dialog.setObjectName("aiSettingsDialog");
  dialog.setWindowTitle("AI Connection Settings");
  auto *layout = new QVBoxLayout(&dialog);
  layout->setContentsMargins(14, 14, 14, 14);
  layout->setSpacing(10);

  auto *label = new QLabel("AMR MCP chat service");
  label->setObjectName("aiSettingsLabel");
  auto *service_edit = new QLineEdit(service_name_);
  service_edit->setObjectName("aiEndpointInput");
  auto *provider_edit = new QLineEdit(providerId());
  provider_edit->setObjectName("aiEndpointInput");
  auto *robot_edit = new QLineEdit(robot_id_);
  robot_edit->setObjectName("aiEndpointInput");
  auto *frame_edit = new QLineEdit(default_frame_);
  frame_edit->setObjectName("aiEndpointInput");
  layout->addWidget(label);
  layout->addWidget(service_edit);
  layout->addWidget(new QLabel("Provider"));
  layout->addWidget(provider_edit);
  layout->addWidget(new QLabel("Robot ID"));
  layout->addWidget(robot_edit);
  layout->addWidget(new QLabel("Default frame"));
  layout->addWidget(frame_edit);

  auto *button_row = new QWidget;
  auto *button_layout = new QHBoxLayout(button_row);
  button_layout->setContentsMargins(0, 0, 0, 0);
  button_layout->addStretch(1);
  auto *apply_button = new QPushButton("Apply");
  apply_button->setObjectName("aiConnectButton");
  auto *close_button = new QPushButton("Close");
  close_button->setObjectName("neutralButton");
  button_layout->addWidget(apply_button);
  button_layout->addWidget(close_button);
  layout->addWidget(button_row);

  connect(apply_button, &QPushButton::clicked, this, [this, service_edit, provider_edit, robot_edit, frame_edit]() {
    service_name_ = service_edit->text().trimmed();
    provider_label_ = provider_edit->text().trimmed().isEmpty() ? "Ollama" : provider_edit->text().trimmed();
    robot_id_ = robot_edit->text().trimmed().isEmpty() ? "burger1" : robot_edit->text().trimmed();
    default_frame_ = frame_edit->text().trimmed().isEmpty() ? "map" : frame_edit->text().trimmed();
    updateConnectionUi();
    Q_EMIT chatServiceNameChanged(service_name_);
    addTranscriptMessage("system", QString("MCP service 설정: %1").arg(service_name_));
  });
  connect(close_button, &QPushButton::clicked, &dialog, &QDialog::accept);

  dialog.exec();
}

/// @copydoc AiChatPanel::sendMessage
void AiChatPanel::sendMessage()
{
  const QString message = input_->toPlainText().trimmed();
  if (message.isEmpty()) {
    return;
  }

  addTranscriptMessage("user", message);
  input_->clear();
  if (!service_available_) {
    addTranscriptMessage("system", QString::fromUtf8("MCP service 상태를 확인하는 중입니다."));
  }
  Q_EMIT chatRequested(providerId(), robot_id_, default_frame_, message);
}

/// @copydoc AiChatPanel::clearTranscript
void AiChatPanel::clearTranscript()
{
  while (QLayoutItem *item = transcript_layout_->takeAt(0)) {
    if (QWidget *widget = item->widget()) {
      widget->deleteLater();
    }
    delete item;
  }

  addTranscriptMessage(
    "system",
    QString::fromUtf8(
      "로봇에게 보낼 요청을 입력하면 MCP 서버로 전달되고, 주행 상태도 이 타임라인에서 함께 볼 수 있습니다."));
}

/// @copydoc AiChatPanel::addTranscriptMessage
void AiChatPanel::addTranscriptMessage(const QString &role, const QString &message)
{
  auto *row = new QWidget;
  row->setObjectName("transcriptRow");
  auto *row_layout = new QHBoxLayout(row);
  row_layout->setContentsMargins(0, 0, 0, 0);
  row_layout->setSpacing(0);

  auto *bubble = new QLabel;
  bubble->setObjectName("transcriptBubble");
  bubble->setProperty("role", role);
  bubble->setWordWrap(true);
  bubble->setTextInteractionFlags(Qt::TextSelectableByMouse);

  QString role_label = "SYSTEM";
  if (role == "user") {
    role_label = "YOU";
  } else if (role == "assistant") {
    role_label = provider_label_;
  }
  bubble->setText(QString("%1\n%2").arg(role_label, message));
  bubble->setMaximumWidth(680);

  if (role == "user") {
    row_layout->addStretch(1);
    row_layout->addWidget(bubble);
  } else {
    row_layout->addWidget(bubble);
    row_layout->addStretch(1);
  }

  transcript_layout_->addWidget(row);
  const auto scroll_to_message = [this, row]() {
      transcript_scroll_->setFocus(Qt::OtherFocusReason);
      transcript_scroll_->ensureWidgetVisible(row, 0, 12);
      auto *bar = transcript_scroll_->verticalScrollBar();
      bar->setValue(bar->maximum());
    };
  QTimer::singleShot(0, this, scroll_to_message);
  QTimer::singleShot(30, this, scroll_to_message);
}

/// @copydoc AiChatPanel::applyPrompt
void AiChatPanel::applyPrompt(const QString &prompt)
{
  input_->setPlainText(prompt);
  input_->setFocus(Qt::OtherFocusReason);
  QTextCursor cursor = input_->textCursor();
  cursor.movePosition(QTextCursor::End);
  input_->setTextCursor(cursor);
}

/// @copydoc AiChatPanel::handleChatResponse
void AiChatPanel::handleChatResponse(
  bool accepted,
  bool command_executed,
  const QString &command_type,
  const QString &response,
  const QString &request_id,
  const QString &error_message)
{
  service_available_ = !request_id.isEmpty();
  updateConnectionUi();
  const QString prefix = QString("[%1 | %2%3]")
    .arg(request_id, command_type, command_executed ? " executed" : "");
  if (!accepted && !error_message.isEmpty()) {
    addTranscriptMessage("system", QString("%1 %2").arg(prefix, error_message));
    return;
  }
  addTranscriptMessage("assistant", QString("%1\n%2").arg(prefix, response));
}

/// @copydoc AiChatPanel::handleMcpFeedback
void AiChatPanel::handleMcpFeedback(const QString &message)
{
  if (message.trimmed().isEmpty()) {
    return;
  }
  addTranscriptMessage("system", QString("MCP EVENT\n%1").arg(message.trimmed()));
}

/// @copydoc AiChatPanel::setServiceAvailable
void AiChatPanel::setServiceAvailable(bool available)
{
  if (service_available_ == available) {
    return;
  }
  service_available_ = available;
  updateConnectionUi();
}

/// @copydoc AiChatPanel::updateConnectionUi
void AiChatPanel::updateConnectionUi()
{
  status_pill_->setText(service_available_ ? "ONLINE" : "OFFLINE");
  status_pill_->setProperty("state", service_available_ ? "online" : "offline");
  status_pill_->style()->unpolish(status_pill_);
  status_pill_->style()->polish(status_pill_);

  summary_label_->setText(
    QString::fromUtf8("로봇 %1 | %2 | MCP %3 | %4")
      .arg(
        robot_id_,
        default_frame_,
        service_available_ ? QString::fromUtf8("서비스 준비됨") : service_name_,
        provider_label_));
  if (send_button_) {
    send_button_->setText(QString::fromUtf8("%1에게 보내기").arg(provider_label_));
  }
}

/// @copydoc AiChatPanel::providerId
QString AiChatPanel::providerId() const
{
  return provider_label_.toLower();
}

}  // namespace amr::visualization
