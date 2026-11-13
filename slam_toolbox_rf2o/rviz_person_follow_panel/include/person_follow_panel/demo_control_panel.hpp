#ifndef PERSON_FOLLOW_PANEL__DEMO_CONTROL_PANEL_HPP_
#define PERSON_FOLLOW_PANEL__DEMO_CONTROL_PANEL_HPP_

#include <memory>
#include <string>
#include <thread>

#include <QPushButton>
#include <QLabel>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace person_follow_panel
{

/// RViz2 panel with one button per person-follow demo service.
///
/// Services (std_srvs/srv/Trigger):
///   /demo/start
///   /demo/stop
///   /demo/clear_target
///   /demo/return_home
class DemoControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DemoControlPanel(QWidget * parent = nullptr);
  ~DemoControlPanel() override;

private Q_SLOTS:
  void onStart();
  void onStop();
  void onClearTarget();
  void onReturnHome();

Q_SIGNALS:
  void statusMessage(const QString & text);

private:
  using Trigger = std_srvs::srv::Trigger;

  void buildUi();
  void callService(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const QString & label);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_client_;
  rclcpp::Client<Trigger>::SharedPtr clear_client_;
  rclcpp::Client<Trigger>::SharedPtr home_client_;

  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QPushButton * clear_button_{nullptr};
  QPushButton * home_button_{nullptr};
  QLabel * status_label_{nullptr};
};

}  // namespace person_follow_panel

#endif  // PERSON_FOLLOW_PANEL__DEMO_CONTROL_PANEL_HPP_
