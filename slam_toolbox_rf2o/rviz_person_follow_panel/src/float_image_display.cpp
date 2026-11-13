#include "person_follow_panel/float_image_display.hpp"

#include <QGuiApplication>
#include <QRect>
#include <QScreen>
#include <QTimer>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/panel_dock_widget.hpp>

namespace person_follow_panel
{

void FloatImageDisplay::onInitialize()
{
  rviz_default_plugins::displays::ImageDisplay::onInitialize();

  // Give RViz time to create and dock the associated render panel before
  // detaching it. 0 ms is usually enough, but 200 ms is safer on a loaded
  // board where the Qt event loop starts slowly.
  QTimer::singleShot(200, this, [this]() { floatPanel(); });
}

void FloatImageDisplay::floatPanel()
{
  auto * panel = getAssociatedWidgetPanel();
  if (!panel) {
    RVIZ_COMMON_LOG_WARNING("FloatImageDisplay: no associated panel");
    return;
  }

  QScreen * screen = QGuiApplication::primaryScreen();
  if (!screen) {
    RVIZ_COMMON_LOG_WARNING("FloatImageDisplay: no primary screen");
    return;
  }

  const QRect available = screen->availableGeometry();
  const int width = available.width() / 2;
  const int height = available.height() / 2;

  panel->setFloating(true);
  panel->resize(width, height);
  panel->move(available.right() - width - 8, available.top() + 8);
  panel->show();

  RVIZ_COMMON_LOG_INFO_STREAM(
    "FloatImageDisplay: floating image window " << width << "x" << height);
}

}  // namespace person_follow_panel

PLUGINLIB_EXPORT_CLASS(person_follow_panel::FloatImageDisplay, rviz_common::Display)
