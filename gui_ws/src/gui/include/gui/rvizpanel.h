#ifndef RVIZPANEL_H
#define RVIZPANEL_H

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QGridLayout>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

namespace rviz_common
{
class Display;
class RenderPanel;
class VisualizationManager;
}

class RvizPanel : public QMainWindow, public rviz_common::WindowManagerInterface
{
    Q_OBJECT
public:
  RvizPanel(
    QApplication *app,
    QWidget * parent = 0);

  QWidget * getParentWindow() override;
  rviz_common::PanelDockWidget * addPane(
    const QString & name,
    QWidget * pane,
    Qt::DockWidgetArea area,
    bool floating) override;

  QWidget * initializeWidget(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);
  void setStatus(const QString & message) override;

  void DisplayGrid();

private slots:
  void setThickness(int thickness_percent);
  void setCellSize(int cell_size_percent);

private:
  void initializeRViz();

  QApplication * app_;
  QWidget * rviz_widget;
  QVBoxLayout * main_layout;

  rviz_common::RenderPanel * render_panel_;
  rviz_common::Display * grid_;
  rviz_common::VisualizationManager * manager_;

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

#endif // RVIZPANEL_H
