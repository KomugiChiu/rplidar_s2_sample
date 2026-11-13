#ifndef PERSON_FOLLOW_PANEL__FLOAT_IMAGE_DISPLAY_HPP_
#define PERSON_FOLLOW_PANEL__FLOAT_IMAGE_DISPLAY_HPP_

#include <rviz_default_plugins/displays/image/image_display.hpp>

namespace person_follow_panel
{

/// Image display that detaches its render panel into a floating window and
/// sizes it to 1/4 of the screen (half width x half height) on startup.
class FloatImageDisplay : public rviz_default_plugins::displays::ImageDisplay
{
  Q_OBJECT

public:
  FloatImageDisplay() = default;
  ~FloatImageDisplay() override = default;

  void onInitialize() override;

private:
  void floatPanel();
};

}  // namespace person_follow_panel

#endif  // PERSON_FOLLOW_PANEL__FLOAT_IMAGE_DISPLAY_HPP_
