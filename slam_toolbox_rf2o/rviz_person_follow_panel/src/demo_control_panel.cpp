#include "person_follow_panel/demo_control_panel.hpp"

#include <chrono>
#include <exception>
#include <memory>

#include <QFont>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace person_follow_panel
{

DemoControlPanel::DemoControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();

  connect(
    this, &DemoControlPanel::statusMessage, this,
    [this](const QString & text) {
      if (status_label_) {
        status_label_->setText(text);
      }
    });

  if (status_label_) {
    status_label_->setText(tr("Ready"));
  }
}

void DemoControlPanel::onInitialize()
{
  auto * context = getDisplayContext();
  if (!context) {
    Q_EMIT statusMessage(tr("RViz display context is not available"));
    return;
  }

  auto abstraction = context->getRosNodeAbstraction().lock();
  if (!abstraction) {
    Q_EMIT statusMessage(tr("RViz ROS node is not available"));
    return;
  }

  node_ = abstraction->get_raw_node();
  if (!node_) {
    Q_EMIT statusMessage(tr("RViz ROS node is not available"));
    return;
  }

  start_client_ = node_->create_client<Trigger>("/demo/start");
  stop_client_ = node_->create_client<Trigger>("/demo/stop");
  clear_client_ = node_->create_client<Trigger>("/demo/clear_target");
  home_client_ = node_->create_client<Trigger>("/demo/return_home");

  Q_EMIT statusMessage(tr("Ready"));
}

void DemoControlPanel::buildUi()
{
  auto * root = new QVBoxLayout(this);

  auto * title = new QLabel(tr("Person Follow"), this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  root->addWidget(title);

  auto * grid = new QGridLayout();

  start_button_ = new QPushButton(tr("Start Follow"), this);
  stop_button_ = new QPushButton(tr("Stop"), this);
  clear_button_ = new QPushButton(tr("Clear Target"), this);
  home_button_ = new QPushButton(tr("Return Home"), this);

  start_button_->setObjectName("start_follow_button");
  stop_button_->setObjectName("stop_follow_button");
  clear_button_->setObjectName("clear_target_button");
  home_button_->setObjectName("return_home_button");

  start_button_->setToolTip(tr("/demo/start (std_srvs/Trigger)"));
  stop_button_->setToolTip(tr("/demo/stop (std_srvs/Trigger)"));
  clear_button_->setToolTip(tr("/demo/clear_target (std_srvs/Trigger)"));
  home_button_->setToolTip(tr("/demo/return_home (std_srvs/Trigger)"));

  grid->addWidget(start_button_, 0, 0);
  grid->addWidget(stop_button_, 0, 1);
  grid->addWidget(clear_button_, 1, 0);
  grid->addWidget(home_button_, 1, 1);
  root->addLayout(grid);

  status_label_ = new QLabel(tr("Ready"), this);
  status_label_->setWordWrap(true);
  status_label_->setObjectName("demo_status_label");
  root->addWidget(status_label_);

  connect(start_button_, &QPushButton::clicked, this, &DemoControlPanel::onStart);
  connect(stop_button_, &QPushButton::clicked, this, &DemoControlPanel::onStop);
  connect(clear_button_, &QPushButton::clicked, this, &DemoControlPanel::onClearTarget);
  connect(home_button_, &QPushButton::clicked, this, &DemoControlPanel::onReturnHome);
}

void DemoControlPanel::onStart()
{
  callService(start_client_, tr("Start Follow"));
}

void DemoControlPanel::onStop()
{
  callService(stop_client_, tr("Stop"));
}

void DemoControlPanel::onClearTarget()
{
  callService(clear_client_, tr("Clear Target"));
}

void DemoControlPanel::onReturnHome()
{
  callService(home_client_, tr("Return Home"));
}

void DemoControlPanel::callService(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  const QString & label)
{
  if (!client) {
    Q_EMIT statusMessage(tr("%1: client is not ready").arg(label));
    return;
  }

  // Give discovery a short chance, but never block the UI for long.
  if (!client->service_is_ready()) {
    client->wait_for_service(std::chrono::milliseconds(200));
  }
  if (!client->service_is_ready()) {
    Q_EMIT statusMessage(tr("%1: service not available").arg(label));
    return;
  }

  Q_EMIT statusMessage(tr("%1: calling...").arg(label));

  auto request = std::make_shared<Trigger::Request>();
  client->async_send_request(
    request,
    [this, label](rclcpp::Client<Trigger>::SharedFuture future) {
      QString text;
      try {
        auto response = future.get();
        if (!response) {
          text = tr("%1: no response").arg(label);
        } else if (response->success) {
          text = tr("%1: OK").arg(label);
          if (!response->message.empty()) {
            text += " - " + QString::fromStdString(response->message);
          }
        } else {
          text = tr("%1: FAILED").arg(label);
          if (!response->message.empty()) {
            text += " - " + QString::fromStdString(response->message);
          }
        }
      } catch (const std::exception & e) {
        text = tr("%1: exception: %2").arg(label, QString::fromUtf8(e.what()));
      } catch (...) {
        text = tr("%1: unknown exception").arg(label);
      }
      Q_EMIT statusMessage(text);
    });
}

}  // namespace person_follow_panel

PLUGINLIB_EXPORT_CLASS(person_follow_panel::DemoControlPanel, rviz_common::Panel)
