#ifndef my_namespace__my_plugin_H
#define my_namespace__my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <talos_wbc_gui/ui_talos_wbc_gui.h>
#include <QWidget>

namespace talos_wbc_gui {

class TalosWBCGui
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  TalosWBCGui();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui::TalosWBCGuiWidget ui_;
  QWidget* widget_;
};
} // namespace
#endif // talos_wbc_gui