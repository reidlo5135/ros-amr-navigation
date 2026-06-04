#include "amr_visualization/main_window.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QDateTime>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QJsonDocument>
#include <QJsonObject>
#include <QListWidget>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QSize>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>

#include <cmath>

namespace amr::visualization
{

namespace
{

QLabel *make_value_label(const QString &text = "--")
{
  auto *label = new QLabel(text);
  label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return label;
}

QFrame *line()
{
  auto *frame = new QFrame;
  frame->setFrameShape(QFrame::HLine);
  frame->setObjectName("separator");
  return frame;
}

QPixmap make_layer_icon(const QString &name, const QSize &size)
{
  QPixmap pixmap(size);
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const auto pen = [&](const QColor &color, qreal width = 1.6) {
    painter.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.setBrush(Qt::NoBrush);
  };

  if (name == "grid") {
    pen(QColor("#5f83ff"), 1.4);
    for (int pos : {4, 8, 12}) {
      painter.drawLine(QPointF(pos, 2), QPointF(pos, 14));
      painter.drawLine(QPointF(2, pos), QPointF(14, pos));
    }
  } else if (name == "map") {
    pen(QColor("#b57cff"), 2.1);
    painter.drawPolyline(QPolygonF{
      QPointF(2.5, 11.0), QPointF(5.0, 8.5), QPointF(8.0, 11.0),
      QPointF(11.0, 5.0), QPointF(14.0, 7.0)});
    pen(QColor("#b57cff"), 1.2);
    painter.drawLine(QPointF(5.0, 2.5), QPointF(5.0, 13.5));
    painter.drawLine(QPointF(11.0, 2.5), QPointF(11.0, 13.5));
  } else if (name == "global_costmap") {
    pen(QColor("#00e4ef"), 1.7);
    painter.drawRect(QRectF(3.0, 3.0, 10.0, 10.0));
    painter.fillRect(QRectF(5.0, 5.0, 2.0, 2.0), QColor("#00e4ef"));
    painter.fillRect(QRectF(9.0, 5.0, 2.0, 2.0), QColor("#00e4ef"));
    painter.fillRect(QRectF(5.0, 9.0, 2.0, 2.0), QColor("#00e4ef"));
    painter.fillRect(QRectF(9.0, 9.0, 2.0, 2.0), QColor("#00e4ef"));
  } else if (name == "local_costmap") {
    pen(QColor("#ff3cf7"), 1.7);
    painter.drawRect(QRectF(4.5, 4.5, 7.0, 7.0));
    painter.fillRect(QRectF(7.0, 7.0, 2.0, 2.0), QColor("#ff3cf7"));
  } else if (name == "footprint") {
    pen(QColor("#65a4ff"), 2.1);
    painter.drawPolyline(QPolygonF{QPointF(3.0, 6.0), QPointF(7.0, 8.5), QPointF(13.0, 4.5)});
    painter.drawLine(QPointF(7.0, 8.5), QPointF(9.0, 12.0));
  } else if (name == "robot") {
    pen(QColor("#cfd9e6"), 1.5);
    painter.drawRect(QRectF(6.0, 3.0, 4.0, 10.0));
    painter.drawRect(QRectF(3.0, 6.0, 10.0, 4.0));
    pen(QColor("#8fa4c3"), 1.4);
    painter.drawLine(QPointF(2.0, 8.0), QPointF(5.0, 8.0));
    painter.drawLine(QPointF(11.0, 8.0), QPointF(14.0, 8.0));
    painter.drawLine(QPointF(8.0, 2.0), QPointF(8.0, 5.0));
    painter.drawLine(QPointF(8.0, 11.0), QPointF(8.0, 14.0));
  } else if (name == "global_plan") {
    pen(QColor("#10c8ff"), 1.8);
    painter.drawPolyline(QPolygonF{
      QPointF(2.5, 12.5), QPointF(5.5, 9.5), QPointF(7.5, 10.0),
      QPointF(10.5, 6.0), QPointF(13.5, 3.5)});
    painter.setBrush(QColor("#10c8ff"));
    painter.drawEllipse(QPointF(2.5, 12.5), 1.3, 1.3);
    painter.drawEllipse(QPointF(13.5, 3.5), 1.3, 1.3);
  } else if (name == "local_plan") {
    pen(QColor("#8cff3a"), 1.8);
    painter.drawPolyline(QPolygonF{
      QPointF(2.5, 11.5), QPointF(5.0, 10.5), QPointF(6.5, 7.5),
      QPointF(9.5, 7.0), QPointF(12.5, 4.0)});
    painter.setBrush(QColor("#8cff3a"));
    painter.drawEllipse(QPointF(2.5, 11.5), 1.2, 1.2);
    painter.drawEllipse(QPointF(12.5, 4.0), 1.2, 1.2);
  } else if (name == "laser_scan") {
    pen(QColor("#66ff54"), 1.5);
    painter.drawArc(QRectF(4.0, 5.0, 12.0, 12.0), 90 * 16, 90 * 16);
    painter.drawLine(QPointF(4.0, 11.0), QPointF(10.0, 11.0));
    painter.drawLine(QPointF(4.0, 11.0), QPointF(8.0, 7.0));
    painter.setBrush(QColor("#66ff54"));
    painter.drawEllipse(QPointF(4.0, 11.0), 1.3, 1.3);
  } else if (name == "tf") {
    pen(QColor("#b47cff"), 1.6);
    painter.drawLine(QPointF(5.0, 12.0), QPointF(5.0, 4.0));
    painter.drawLine(QPointF(5.0, 12.0), QPointF(12.0, 12.0));
    pen(QColor("#b47cff"), 1.3);
    painter.drawLine(QPointF(5.0, 4.0), QPointF(3.0, 6.0));
    painter.drawLine(QPointF(5.0, 4.0), QPointF(7.0, 6.0));
    painter.drawLine(QPointF(12.0, 12.0), QPointF(10.0, 10.0));
    painter.drawLine(QPointF(12.0, 12.0), QPointF(10.0, 14.0));
  } else if (name == "settings") {
    pen(QColor("#8fa4c3"), 1.4);
    painter.drawEllipse(QPointF(8.0, 8.0), 3.2, 3.2);
    painter.drawEllipse(QPointF(8.0, 8.0), 1.1, 1.1);
    for (const QPointF &p : {
        QPointF(8.0, 2.0), QPointF(8.0, 14.0), QPointF(2.0, 8.0), QPointF(14.0, 8.0)}) {
      painter.drawPoint(p);
    }
  }

  return pixmap;
}

QString pose_text(const Pose2D &pose)
{
  if (!pose.valid) {
    return "Pose x --, y --, z --";
  }
  return QString("Pose x %1, y %2, z %3")
    .arg(pose.x, 0, 'f', 2)
    .arg(pose.y, 0, 'f', 2)
    .arg(pose.z, 0, 'f', 2);
}

void set_label_if_changed(QLabel *label, const QString &text)
{
  if (label && label->text() != text) {
    label->setText(text);
  }
}

QString summarize_event_payload(const QString &event)
{
  const auto document = QJsonDocument::fromJson(event.toUtf8());
  if (!document.isObject()) {
    return event;
  }

  const auto object = document.object();
  const QString event_type = object.value("event_type").toString();
  const QString runtime_state = object.value("runtime_state").toString();
  const QString blocked_context = object.value("blocked_context").toString();
  const QString recovery_phase = object.value("recovery_phase").toString();
  const bool local_escape_active = object.value("local_escape_active").toBool(false);
  const QString action_status_label = object.value("action_status_label").toString();
  const int current_goal_index = object.value("current_goal_index").toInt(0);
  const int goal_count = object.value("goal_count").toInt(0);
  const int recoveries = object.value("number_of_recoveries").toInt(0);

  QStringList parts;
  if (!event_type.isEmpty()) {
    parts << event_type;
  }
  if (!runtime_state.isEmpty()) {
    parts << runtime_state;
  }
  if (!action_status_label.isEmpty()) {
    parts << action_status_label;
  }
  if (goal_count > 0) {
    parts << QString("goal %1/%2").arg(current_goal_index + 1).arg(goal_count);
  }
  if (!blocked_context.isEmpty() && blocked_context != "clear") {
    parts << blocked_context;
  }
  if (!recovery_phase.isEmpty() && recovery_phase != "idle") {
    parts << recovery_phase;
  }
  if (local_escape_active) {
    parts << "local_escape";
  }
  if (recoveries > 0) {
    parts << QString("recoveries %1").arg(recoveries);
  }

  if (parts.isEmpty()) {
    return event;
  }
  return parts.join(" | ");
}

}  // namespace

MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent),
  ros_worker_(std::make_unique<RosWorker>())
{
  setWindowTitle("AMR Visualization");
  resize(1500, 900);

  auto *root = new QWidget;
  auto *root_layout = new QVBoxLayout(root);
  root_layout->setContentsMargins(0, 0, 0, 0);
  root_layout->setSpacing(0);
  root_layout->addWidget(makeTopBar());

  auto *content = new QWidget;
  auto *content_layout = new QHBoxLayout(content);
  content_layout->setContentsMargins(0, 0, 0, 0);
  content_layout->setSpacing(6);

  scene_ = new SceneWidget;
  content_layout->addWidget(makeLeftPanel());
  content_layout->addWidget(scene_, 1);
  content_layout->addWidget(makeRightPanel());
  root_layout->addWidget(content, 1);
  setCentralWidget(root);

  connect(scene_, &SceneWidget::waypointsChanged, this, &MainWindow::updateWaypointList);
  connect(scene_, &SceneWidget::visualizationEvent, this, &MainWindow::appendEvent, Qt::QueuedConnection);
  connect(scene_, &SceneWidget::selectedWaypointChanged, this, [this](int index) {
    if (!waypoint_list_) {
      return;
    }
    waypoint_list_->setCurrentRow(index);
  });

  connect(ros_worker_.get(), &RosWorker::connectionStateChanged, this, [this](const QString &state) {
    appendEvent(state);
  }, Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::mapChanged, scene_, &SceneWidget::setMap, Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::tfFramesChanged, scene_, &SceneWidget::setTfFrames, Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::robotModelChanged,
    scene_, &SceneWidget::setRobotModel,
    Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::globalCostmapChanged,
    scene_, &SceneWidget::setGlobalCostmap,
    Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::localCostmapChanged,
    scene_, &SceneWidget::setLocalCostmap,
    Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::scanChanged, scene_, &SceneWidget::setScan, Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::robotPoseChanged,
    scene_, &SceneWidget::setRobotPose,
    Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::robotPoseChanged, this, &MainWindow::updateAimPose, Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::globalPathChanged,
    scene_, &SceneWidget::setGlobalPath,
    Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::localPathChanged,
    scene_, &SceneWidget::setLocalPath,
    Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::motionStatusChanged,
    this, &MainWindow::updateMotionStatus,
    Qt::QueuedConnection);
  connect(
    ros_worker_.get(), &RosWorker::runtimeSummaryChanged,
    this, &MainWindow::updateRuntimeSummary,
    Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::batteryStateChanged, this, [this](double percentage, bool present) {
    auto refresh_battery_style = [this](const QString &level) {
        battery_bar_->setProperty("level", level);
        battery_label_->setProperty("level", level);
        battery_bar_->style()->unpolish(battery_bar_);
        battery_bar_->style()->polish(battery_bar_);
        battery_label_->style()->unpolish(battery_label_);
        battery_label_->style()->polish(battery_label_);
      };
    if (!present || percentage < 0.0) {
      battery_bar_->setValue(0);
      battery_label_->setText("--%");
      refresh_battery_style("unknown");
      return;
    }
    const int rounded = std::clamp(static_cast<int>(std::round(percentage)), 0, 100);
    battery_bar_->setValue(rounded);
    battery_label_->setText(QString("%1%").arg(rounded));
    if (rounded >= 80) {
      refresh_battery_style("high");
    } else if (rounded >= 30) {
      refresh_battery_style("mid");
    } else {
      refresh_battery_style("low");
    }
  }, Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::goalStateChanged, this, [this](const QString &state) {
    goal_label_->setText(state);
  }, Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::navigationCompleted, this, [this](bool succeeded) {
    setWaypointEditingLocked(false);
    if (succeeded) {
      scene_->clearNavigationOverlays();
      appendEvent("Navigation overlays cleared");
    }
  }, Qt::QueuedConnection);
  connect(ros_worker_.get(), &RosWorker::eventReceived, this, &MainWindow::appendEvent, Qt::QueuedConnection);

  applyStyle();
  ros_worker_->start();
}

MainWindow::~MainWindow()
{
  ros_worker_->stop();
}

QWidget *MainWindow::makeTopBar()
{
  auto *bar = new QWidget;
  bar->setObjectName("topBar");
  auto *layout = new QHBoxLayout(bar);
  layout->setContentsMargins(8, 5, 8, 5);
  layout->setSpacing(8);

  auto *title = new QLabel("AMR");
  title->setObjectName("brandLabel");
  frame_label_ = new QLabel("Fixed Frame: map");
  frame_label_->setObjectName("pill");
  aim_label_ = new QLabel("Pose x --, y --, z --");
  aim_label_->setObjectName("pill");
  mode_label_ = new QLabel("MANUAL");
  mode_label_->setObjectName("modePill");
  ai_label_ = new QLabel("AI");
  ai_label_->setObjectName("statusPill");
  auto *battery_pill = new QWidget;
  battery_pill->setObjectName("batteryPill");
  auto *battery_layout = new QHBoxLayout(battery_pill);
  battery_layout->setContentsMargins(10, 3, 10, 3);
  battery_layout->setSpacing(8);
  battery_bar_ = new QProgressBar;
  battery_bar_->setObjectName("batteryBar");
  battery_bar_->setRange(0, 100);
  battery_bar_->setValue(0);
  battery_bar_->setTextVisible(false);
  battery_bar_->setFixedSize(30, 12);
  battery_label_ = new QLabel("--%");
  battery_label_->setObjectName("batteryText");
  battery_layout->addWidget(battery_bar_);
  battery_layout->addWidget(battery_label_);

  layout->addWidget(title);
  layout->addStretch(1);
  layout->addWidget(frame_label_);
  layout->addWidget(aim_label_);
  layout->addWidget(mode_label_);
  layout->addWidget(ai_label_);
  layout->addWidget(battery_pill);
  return bar;
}

QWidget *MainWindow::makeLeftPanel()
{
  auto *panel = new QWidget;
  panel->setObjectName("sidePanel");
  panel->setFixedWidth(268);
  auto *layout = new QVBoxLayout(panel);
  layout->setContentsMargins(6, 10, 10, 10);
  layout->setSpacing(8);

  auto *command_panel = new QWidget;
  auto *command_layout = new QVBoxLayout(command_panel);
  command_layout->setContentsMargins(0, 0, 0, 0);
  command_layout->setSpacing(8);
  auto *command = new QLabel("COMMAND");
  command->setObjectName("sectionTitle");
  command_layout->addWidget(command);

  add_waypoint_button_ = new QPushButton("+ Add Waypoint");
  add_waypoint_button_->setObjectName("routeButton");
  add_waypoint_button_->setCheckable(true);
  connect(add_waypoint_button_, &QPushButton::toggled, scene_, &SceneWidget::setAddWaypointMode);
  command_layout->addWidget(add_waypoint_button_);

  waypoint_list_ = new QListWidget;
  waypoint_list_->setObjectName("waypointList");
  waypoint_list_->setMinimumHeight(120);
  waypoint_list_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  command_layout->addWidget(waypoint_list_, 1);
  connect(waypoint_list_, &QListWidget::currentRowChanged, scene_, &SceneWidget::selectWaypoint);

  auto *waypoint_tools = new QHBoxLayout;
  auto *move_up = new QPushButton("Up");
  auto *move_down = new QPushButton("Down");
  auto *remove = new QPushButton("Delete");
  move_up->setObjectName("compactButton");
  move_down->setObjectName("compactButton");
  remove->setObjectName("compactDangerButton");
  waypoint_tools->addWidget(move_up);
  waypoint_tools->addWidget(move_down);
  waypoint_tools->addWidget(remove);
  command_layout->addLayout(waypoint_tools);
  connect(move_up, &QPushButton::clicked, scene_, &SceneWidget::moveSelectedWaypointUp);
  connect(move_down, &QPushButton::clicked, scene_, &SceneWidget::moveSelectedWaypointDown);
  connect(remove, &QPushButton::clicked, scene_, &SceneWidget::removeSelectedWaypoint);

  auto *command_grid = new QGridLayout;
  send_button_ = new QPushButton("Send");
  send_button_->setObjectName("sendButton");
  auto *cancel_button = new QPushButton("Cancel");
  cancel_button->setObjectName("cancelButton");
  auto *clear_button = new QPushButton("Clear");
  clear_button->setObjectName("neutralButton");
  auto *initial_pose_button = new QPushButton("Init Pose");
  initial_pose_button->setObjectName("initButton");
  command_grid->addWidget(send_button_, 0, 0);
  command_grid->addWidget(cancel_button, 0, 1);
  command_grid->addWidget(clear_button, 1, 0);
  command_grid->addWidget(initial_pose_button, 1, 1);
  command_layout->addLayout(command_grid);
  connect(send_button_, &QPushButton::clicked, this, &MainWindow::sendGoal);
  connect(cancel_button, &QPushButton::clicked, this, [this]() {
    ros_worker_->cancelNavigation();
    setWaypointEditingLocked(false);
  });
  connect(clear_button, &QPushButton::clicked, scene_, &SceneWidget::clearSchedule);
  connect(initial_pose_button, &QPushButton::clicked, this, [this]() {
    ros_worker_->publishInitialPose(scene_->aimPose());
  });

  layout->addWidget(command_panel, 3);
  layout->addWidget(line());
  auto *visualization_panel = new QWidget;
  auto *visualization_layout = new QVBoxLayout(visualization_panel);
  visualization_layout->setContentsMargins(0, 0, 0, 0);
  visualization_layout->setSpacing(6);
  auto *visualization_header = new QWidget;
  auto *visualization_header_layout = new QHBoxLayout(visualization_header);
  visualization_header_layout->setContentsMargins(0, 0, 0, 0);
  visualization_header_layout->setSpacing(0);
  auto *visualization = new QLabel("VISUALIZATION");
  visualization->setObjectName("sectionTitle");
  auto *visualization_settings = new QToolButton;
  visualization_settings->setObjectName("panelIconButton");
  visualization_settings->setIcon(QIcon(make_layer_icon("settings", QSize(16, 16))));
  visualization_settings->setIconSize(QSize(16, 16));
  visualization_settings->setAutoRaise(true);
  visualization_header_layout->addWidget(visualization);
  visualization_header_layout->addStretch(1);
  visualization_header_layout->addWidget(visualization_settings);
  visualization_layout->addWidget(visualization_header);

  auto grid =
    makeLayerCheckBox("Grid", "grid", true);
  auto map =
    makeLayerCheckBox("Map", "map", true);
  auto global_costmap =
    makeLayerCheckBox("Global Costmap", "global_costmap", true);
  auto local_costmap =
    makeLayerCheckBox("Local Costmap", "local_costmap", true);
  auto footprint =
    makeLayerCheckBox("Exact Footprint", "footprint", true);
  auto robot =
    makeLayerCheckBox("Robot", "robot", true);
  auto global_path =
    makeLayerCheckBox("Global Plan", "global_plan", true);
  auto local_path =
    makeLayerCheckBox("Local Plan", "local_plan", true);
  auto scan =
    makeLayerCheckBox("LaserScan", "laser_scan", true);
  auto tf =
    makeLayerCheckBox("TF", "tf", true);
  visualization_layout->addWidget(grid.row, 1);
  visualization_layout->addWidget(map.row, 1);
  visualization_layout->addWidget(global_costmap.row, 1);
  visualization_layout->addWidget(local_costmap.row, 1);
  visualization_layout->addWidget(footprint.row, 1);
  visualization_layout->addWidget(robot.row, 1);
  visualization_layout->addWidget(global_path.row, 1);
  visualization_layout->addWidget(local_path.row, 1);
  visualization_layout->addWidget(scan.row, 1);
  visualization_layout->addWidget(tf.row, 1);
  connect(grid.check, &QCheckBox::toggled, scene_, &SceneWidget::setGridVisible);
  connect(map.check, &QCheckBox::toggled, scene_, &SceneWidget::setMapVisible);
  connect(global_costmap.check, &QCheckBox::toggled, scene_, &SceneWidget::setGlobalCostmapVisible);
  connect(local_costmap.check, &QCheckBox::toggled, scene_, &SceneWidget::setLocalCostmapVisible);
  connect(
    global_costmap.check, &QCheckBox::toggled,
    ros_worker_.get(), &RosWorker::setGlobalCostmapSubscriptionEnabled);
  connect(
    local_costmap.check, &QCheckBox::toggled,
    ros_worker_.get(), &RosWorker::setLocalCostmapSubscriptionEnabled);
  connect(scan.check, &QCheckBox::toggled, scene_, &SceneWidget::setScanVisible);
  connect(scan.check, &QCheckBox::toggled, ros_worker_.get(), &RosWorker::setScanSubscriptionEnabled);
  connect(footprint.check, &QCheckBox::toggled, scene_, &SceneWidget::setFootprintVisible);
  connect(robot.check, &QCheckBox::toggled, scene_, &SceneWidget::setRobotVisible);
  connect(tf.check, &QCheckBox::toggled, scene_, &SceneWidget::setTfVisible);
  connect(global_path.check, &QCheckBox::toggled, scene_, &SceneWidget::setGlobalPathVisible);
  connect(local_path.check, &QCheckBox::toggled, scene_, &SceneWidget::setLocalPathVisible);
  visualization_layout->addStretch(1);
  layout->addWidget(visualization_panel, 2);
  return panel;
}

QWidget *MainWindow::makeRightPanel()
{
  auto *panel = new QWidget;
  panel->setObjectName("sidePanel");
  panel->setFixedWidth(278);
  auto *layout = new QVBoxLayout(panel);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(10);

  auto *status_title = new QLabel("NAVIGATION STATUS");
  status_title->setObjectName("sectionTitle");
  layout->addWidget(status_title);

  motion_label_ = make_value_label("Idle");
  remaining_label_ = make_value_label("-- m");
  heading_label_ = make_value_label("-- rad");
  goal_label_ = make_value_label("Idle");
  blocked_label_ = make_value_label("Clear");
  recovery_label_ = make_value_label("idle");
  appendStatusRow(layout, "Motion", motion_label_);
  appendStatusRow(layout, "Remaining", remaining_label_);
  appendStatusRow(layout, "Heading", heading_label_);
  appendStatusRow(layout, "Goal", goal_label_);
  appendStatusRow(layout, "Blocked Source", blocked_label_);
  appendStatusRow(layout, "Recovery", recovery_label_);

  layout->addWidget(line());
  auto *events_title = new QLabel("EVENTS / FEEDBACK");
  events_title->setObjectName("sectionTitle");
  layout->addWidget(events_title);
  event_list_ = new QListWidget;
  event_list_->setObjectName("eventList");
  layout->addWidget(event_list_, 1);

  layout->addWidget(line());
  auto *joystick_title = new QLabel("JOYSTICK");
  joystick_title->setObjectName("sectionTitle");
  layout->addWidget(joystick_title);
  auto *joystick = new QLabel;
  joystick->setObjectName("joystick");
  joystick->setMinimumHeight(150);
  joystick->setAlignment(Qt::AlignCenter);
  joystick->setText("●");
  layout->addWidget(joystick);
  appendStatusRow(layout, "Linear X", make_value_label("0.000 m/s"));
  appendStatusRow(layout, "Angular Z", make_value_label("0.000 rad/s"));
  return panel;
}

MainWindow::LayerCheckRow MainWindow::makeLayerCheckBox(
  const QString &label,
  const QString &icon_name,
  bool checked)
{
  auto *row = new QWidget;
  row->setObjectName("layerRow");
  row->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  auto *row_layout = new QHBoxLayout(row);
  row_layout->setContentsMargins(0, 0, 0, 0);
  row_layout->setSpacing(8);

  auto *check = new QCheckBox;
  check->setChecked(checked);
  check->setObjectName("layerCheckBox");
  check->setFixedSize(14, 18);

  auto *icon = new QLabel;
  icon->setObjectName("layerIcon");
  icon->setFixedSize(16, 16);
  icon->setPixmap(make_layer_icon(icon_name, QSize(16, 16)));

  auto *text = new QLabel(label);
  text->setObjectName("layerLabel");
  text->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);

  row_layout->addWidget(check);
  row_layout->addWidget(icon);
  row_layout->addWidget(text, 1);
  return {row, check};
}

void MainWindow::appendStatusRow(QVBoxLayout *layout, const QString &label, QLabel *value)
{
  auto *row = new QWidget;
  auto *row_layout = new QHBoxLayout(row);
  row_layout->setContentsMargins(0, 0, 0, 0);
  row_layout->addWidget(new QLabel(label));
  row_layout->addStretch(1);
  row_layout->addWidget(value);
  layout->addWidget(row);
}

void MainWindow::updateAimPose(const Pose2D &pose)
{
  set_label_if_changed(aim_label_, pose_text(pose));
}

void MainWindow::updateMotionStatus(const MotionStatusData &status)
{
  QString motion_state = "Idle";
  if (status.goal_reached || status.command_completed) {
    motion_state = "Reached";
  } else if (status.stalled) {
    motion_state = "Stalled";
  } else if (status.blocked) {
    motion_state = "Blocked";
  } else if (status.active) {
    motion_state = "Running";
  }
  const bool navigation_running =
    status.active && !status.goal_reached && !status.command_completed;
  setWaypointEditingLocked(navigation_running);

  set_label_if_changed(motion_label_, motion_state);
  set_label_if_changed(
    remaining_label_, QString("%1 m").arg(status.remaining_distance, 0, 'f', 2));
  set_label_if_changed(
    heading_label_, QString("%1 rad").arg(status.heading_error, 0, 'f', 2));
  set_label_if_changed(blocked_label_, status.blocked ? status.blocked_source : "Clear");
}

void MainWindow::updateRuntimeSummary(const RuntimeSummary &summary)
{
  if (!summary.action_status.isEmpty() && summary.action_status != "unknown") {
    set_label_if_changed(goal_label_, summary.action_status);
  } else if (!summary.runtime_state.isEmpty()) {
    set_label_if_changed(goal_label_, summary.runtime_state);
  }

  if (!summary.blocked_context.isEmpty()) {
    set_label_if_changed(
      blocked_label_, summary.blocked_context == "clear" ? "Clear" : summary.blocked_context);
  }
  set_label_if_changed(recovery_label_, summary.recovery_phase);
}

void MainWindow::updateWaypointList(const QVector<Pose2D> &waypoints)
{
  const int selected = scene_->selectedWaypointIndex();
  waypoint_list_->blockSignals(true);
  waypoint_list_->clear();
  for (int i = 0; i < waypoints.size(); ++i) {
    const auto &pose = waypoints[i];
    waypoint_list_->addItem(
      QString("%1  x %2  y %3  yaw %4")
        .arg(i + 1)
        .arg(pose.x, 0, 'f', 2)
        .arg(pose.y, 0, 'f', 2)
        .arg(pose.yaw, 0, 'f', 2));
  }
  waypoint_list_->setCurrentRow(selected);
  waypoint_list_->blockSignals(false);
}

void MainWindow::appendEvent(const QString &event)
{
  const QString stamp = QDateTime::currentDateTime().toString("HH:mm:ss");
  event_list_->addItem(QString("%1  %2").arg(stamp, summarize_event_payload(event)));
  while (event_list_->count() > 100) {
    delete event_list_->takeItem(0);
  }
  event_list_->scrollToBottom();
}

void MainWindow::sendGoal()
{
  if (add_waypoint_button_ && add_waypoint_button_->isChecked()) {
    add_waypoint_button_->setChecked(false);
  }
  const auto waypoints = scene_->waypoints();
  if (!waypoints.empty()) {
    ros_worker_->sendRoute(waypoints);
    return;
  }
  ros_worker_->sendSingleGoal(scene_->aimPose());
}

void MainWindow::setWaypointEditingLocked(bool locked)
{
  if (waypoint_editing_locked_ == locked) {
    return;
  }
  waypoint_editing_locked_ = locked;
  if (!add_waypoint_button_) {
    return;
  }
  if (locked && add_waypoint_button_->isChecked()) {
    add_waypoint_button_->setChecked(false);
  }
  add_waypoint_button_->setEnabled(!locked);
}

void MainWindow::applyStyle()
{
  qApp->setStyleSheet(R"(
    QMainWindow, QWidget {
      background: #101312;
      color: #c2cfca;
      font-family: "Noto Sans", "DejaVu Sans", sans-serif;
      font-size: 13px;
    }
    #topBar {
      background: #171b1f;
      border-bottom: 1px solid #303a3a;
    }
    #brandLabel {
      color: #f0a321;
      font-weight: 700;
    }
    #sidePanel {
      background: #090c0b;
      border-right: 1px solid #2e3837;
      border-left: 1px solid #2e3837;
    }
    #sectionTitle {
      color: #9fb0ad;
      font-size: 12px;
      font-weight: 700;
    }
    #pill, #modePill, #statusPill, #batteryPill, #topMetric {
      background: #111923;
      border: 1px solid #26384f;
      border-radius: 3px;
      padding: 4px 10px;
      color: #a9b8c6;
    }
    #modePill {
      background: #18315f;
      border-color: #386cc8;
      color: #d8e4ff;
    }
    #statusPill, #batteryPill {
      color: #c3ccd8;
      min-height: 18px;
    }
    #statusPill {
      min-width: 42px;
    }
    #batteryPill {
      min-width: 92px;
      padding: 0px;
    }
    #batteryText {
      color: #cfd7e3;
      min-width: 30px;
    }
    #batteryText[level="high"] {
      color: #62f58b;
    }
    #batteryText[level="mid"] {
      color: #ffd65a;
    }
    #batteryText[level="low"] {
      color: #ff6b6b;
    }
    #batteryText[level="unknown"] {
      color: #cfd7e3;
    }
    #batteryBar {
      border: 1px solid #7f91a8;
      border-radius: 2px;
      background: #0d1218;
      padding: 1px;
    }
    #batteryBar::chunk {
      background: #d7dde6;
      border-radius: 1px;
    }
    #batteryBar[level="high"]::chunk {
      background: #2ee66f;
    }
    #batteryBar[level="mid"]::chunk {
      background: #f0c83a;
    }
    #batteryBar[level="low"]::chunk {
      background: #ef4b4b;
    }
    #batteryBar[level="unknown"]::chunk {
      background: #d7dde6;
    }
    #topMetric {
      min-width: 92px;
    }
    QLabel {
      color: #b8c5c0;
    }
    #inputLike, #commandBox {
      border: 1px solid #3a4c49;
      background: #050807;
      padding: 8px;
      min-height: 20px;
    }
    #commandBox {
      min-height: 50px;
    }
    QPushButton {
      border: 1px solid #40504e;
      border-radius: 5px;
      padding: 8px 12px;
      background: #20252d;
      color: #d8e0dd;
      font-weight: 600;
    }
    QPushButton:hover {
      border-color: #5f7770;
    }
    #routeButton {
      background: #142653;
      border-color: #264b96;
      color: #9fc2ff;
      min-height: 34px;
    }
    #routeButton:hover {
      background: #1a3676;
      border-color: #4b7fdd;
      color: #d9e7ff;
    }
    #routeButton:pressed {
      background: #0d1f45;
      border-color: #79a2f2;
      padding-top: 10px;
      padding-bottom: 6px;
    }
    #routeButton:checked {
      background: #24438e;
      border-color: #86b0ff;
      color: #ffffff;
    }
    #routeButton:checked:hover {
      background: #2d52ab;
      border-color: #a4c4ff;
    }
    #sendButton {
      background: #0c4b21;
      border-color: #1c7d3c;
      color: #75f49d;
    }
    #cancelButton {
      background: #581617;
      border-color: #8a2f32;
      color: #ff8588;
    }
    #initButton {
      background: #0c4a4b;
      border-color: #1a7476;
      color: #64f4ee;
    }
    #subtleButton, #neutralButton {
      background: #20252d;
      color: #b5c1c0;
    }
    #layerRow {
      min-height: 26px;
    }
    #layerRow:hover {
      background: #111817;
    }
    #layerLabel {
      color: #b8c5c0;
      font-size: 13px;
    }
    #layerRow:hover #layerLabel {
      color: #e2ece8;
    }
    #layerIcon {
      background: transparent;
    }
    #layerCheckBox::indicator {
      width: 13px;
      height: 13px;
    }
    #panelIconButton {
      border: none;
      background: transparent;
      padding: 0;
      margin: 0;
      min-width: 20px;
      min-height: 20px;
    }
    #panelIconButton:hover {
      background: #162021;
      border-radius: 3px;
    }
    #cameraControls {
      background: rgba(8, 12, 16, 186);
      border: 1px solid #38505f;
      border-radius: 5px;
    }
    #cameraToolButton {
      background: #132028;
      border: 1px solid #4a6575;
      border-radius: 4px;
      color: #dce8ee;
      font-size: 17px;
      font-weight: 700;
      padding: 0;
    }
    #cameraToolButton:hover {
      background: #1c3340;
      border-color: #79a9bd;
    }
    #cameraToolButton:checked {
      background: #14443d;
      border-color: #45b899;
      color: #eafff7;
    }
    #separator {
      color: #4c5654;
      background: #4c5654;
      max-height: 1px;
    }
    #eventList, #waypointList {
      background: #030505;
      border: 1px solid #32433e;
      border-radius: 4px;
      color: #e5ece8;
    }
    #eventList::item, #waypointList::item {
      padding: 3px;
    }
    #waypointList::item:selected {
      background: #1c355f;
      color: #ffffff;
    }
    #compactButton, #compactDangerButton {
      padding: 5px 8px;
      font-size: 12px;
    }
    #compactDangerButton {
      background: #411215;
      border-color: #753135;
      color: #ff9292;
    }
    #joystick {
      color: #ff9917;
      font-size: 38px;
      background: #08101a;
      border: 1px solid #2e4c80;
      border-radius: 75px;
    }
  )");
}

}  // namespace amr::visualization
