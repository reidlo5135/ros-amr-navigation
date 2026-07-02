#include <QApplication>
#include <QIcon>

/**
 * @file main.cpp
 * @brief Entry point for the AMR Qt visualization application.
 */

#include "amr_visualization/main_window.hpp"
#include "rclcpp/rclcpp.hpp"

/// @brief Initialize Qt/ROS integration, show the main window, and run the app loop.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  qRegisterMetaType<amr::visualization::Pose2D>("amr::visualization::Pose2D");
  qRegisterMetaType<amr::visualization::FrameVisual>("amr::visualization::FrameVisual");
  qRegisterMetaType<QVector<amr::visualization::FrameVisual>>("QVector<amr::visualization::FrameVisual>");
  qRegisterMetaType<amr::visualization::GridMap>("amr::visualization::GridMap");
  qRegisterMetaType<amr::visualization::PathData>("amr::visualization::PathData");
  qRegisterMetaType<amr::visualization::ScanData>("amr::visualization::ScanData");
  qRegisterMetaType<amr::visualization::MotionStatusData>("amr::visualization::MotionStatusData");
  qRegisterMetaType<amr::visualization::RuntimeSummary>("amr::visualization::RuntimeSummary");

  QApplication app(argc, argv);
  app.setApplicationName("AMR Visualization");
  app.setDesktopFileName("amr_visualization");
  const QIcon app_icon = QIcon::fromTheme("amr_visualization");
  app.setWindowIcon(app_icon);
  amr::visualization::MainWindow window;
  window.setWindowIcon(app_icon);
  window.show();
  return app.exec();
}
