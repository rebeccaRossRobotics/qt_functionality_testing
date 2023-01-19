#include "gui/rvizpanel.h"
#include <QGridLayout>
#include <QLabel>
#include <QSlider>

#include "rclcpp/clock.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"

RvizPanel::RvizPanel(
  QApplication * app,
  QWidget * parent)
  : app_(app), QMainWindow(parent)
{
}

QWidget * RvizPanel::initializeWidget(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node
)
{
    rviz_ros_node_ = rviz_ros_node;
    // Construct the layout
    QLabel * thickness_label = new QLabel("Line Thickness");
    QSlider * thickness_slider = new QSlider(Qt::Horizontal);
    thickness_slider->setMinimum(1);
    thickness_slider->setMaximum(100);
    QLabel * cell_size_label = new QLabel("Cell Size");
    QSlider * cell_size_slider = new QSlider(Qt::Horizontal);
    cell_size_slider->setMinimum(1);
    cell_size_slider->setMaximum(100);
    QGridLayout * controls_layout = new QGridLayout();
    controls_layout->addWidget(thickness_label, 0, 0);
    controls_layout->addWidget(thickness_slider, 0, 1);
    controls_layout->addWidget(cell_size_label, 1, 0);
    controls_layout->addWidget(cell_size_slider, 1, 1);

    // Add visualization
    main_layout = new QVBoxLayout;
    main_layout->addLayout(controls_layout);
    rviz_widget = new QWidget();
    main_layout->setSpacing(0);
    main_layout->setMargin(0);

//  // Initialize the classes we need from rviz
    initializeRViz();
   main_layout->addWidget(render_panel_);
   rviz_widget->setLayout(main_layout);
    // rviz_widget = render_panel_;

    // Signals
    connect(thickness_slider, SIGNAL(valueChanged(int)), this, SLOT(setThickness(int)));
    connect(cell_size_slider, SIGNAL(valueChanged(int)), this, SLOT(setCellSize(int)));

    // Display the rviz grid plugin
    DisplayGrid();

    // Intialize the sliders
    thickness_slider->setValue(25);
    cell_size_slider->setValue(10);

    return rviz_widget;
}

QWidget * RvizPanel::getParentWindow()
{
  return this;
}

rviz_common::PanelDockWidget * RvizPanel::addPane(
  const QString & name,
  QWidget * pane,
  Qt::DockWidgetArea area,
  bool floating)
{
  // TODO(mjeronimo)
  return nullptr;
}

void RvizPanel::setStatus(const QString & message)
{
  // TODO(mjeronimo)
}

void RvizPanel::DisplayGrid()
{
  grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
  assert(grid_ != NULL);
  grid_->subProp("Line Style")->setValue("Billboards");
  grid_->subProp("Color")->setValue(QColor(Qt::yellow));
}

void RvizPanel::initializeRViz()
{
  app_->processEvents();
  render_panel_ = new rviz_common::RenderPanel(rviz_widget);
  app_->processEvents();
  render_panel_->getRenderWindow()->initialize();
  auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
  manager_ = new rviz_common::VisualizationManager(render_panel_, rviz_ros_node_, this, clock);
  render_panel_->initialize(manager_);
  app_->processEvents();
  manager_->initialize();
  manager_->startUpdate();
}

void RvizPanel::setThickness(int thickness_percent)
{
  if (grid_ != NULL) {
    grid_->subProp("Line Style")->subProp("Line Width")->setValue(thickness_percent / 100.0f);
  }
}

void RvizPanel::setCellSize(int cell_size_percent)
{
  if (grid_ != NULL) {
    grid_->subProp("Cell Size")->setValue(cell_size_percent / 10.0f);
  }
}
