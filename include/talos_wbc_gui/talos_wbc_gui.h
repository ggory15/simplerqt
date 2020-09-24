#ifndef my_namespace__my_plugin_H
#define my_namespace__my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <talos_wbc_gui/ui_talos_wbc_gui.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <ros/macros.h>

#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QObject>
#include <QStateMachine>
#include <QState>
#include <QEventTransition>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <QMetaType>

#include <QGraphicsRectItem>
#include <QGraphicsSceneWheelEvent>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QStringListModel>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace talos_wbc_gui {

class MyQGraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit MyQGraphicsScene(QWidget *parent = 0);
    virtual void wheelEvent(QGraphicsSceneWheelEvent *event);
    //virtual void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);

public slots:

private:
};

class MyQGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit MyQGraphicsView(QWidget *parent = 0);
    //virtual void wheelEvent(QGraphicsViewWheelEvent *event);
    //virtual void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);

public slots:

private:
};

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

protected slots:
  virtual void timercb(const rosgraph_msgs::ClockConstPtr &msg);
  virtual void jointcb(const sensor_msgs::JointStateConstPtr &msg);
  virtual void forcecb(const geometry_msgs::WrenchStampedConstPtr &msg);
  virtual void imucb(const sensor_msgs::ImuConstPtr &msg);

  virtual void jstatebtn();
  virtual void sstatebtn();

private:
  ros::Time robot_time;
  Eigen::Vector3d imu_rpy;
  Eigen::Quaterniond imu_quat;
  ros::NodeHandle nh_;

public: 
  ros::Subscriber timesub; // for ros time
  ros::Subscriber jointsub; // for joint information
  std::vector<ros::Subscriber> forcesubs; // for force sensor
  ros::Subscriber imusub;
  

signals:
  void TimerCallback(const rosgraph_msgs::ClockConstPtr &msg);
  void JointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
  void ForceSensorCallback(const geometry_msgs::WrenchStampedConstPtr &msg);
  void IMUSensorCallback(const sensor_msgs::ImuConstPtr &msg);

private:
  Ui::TalosWBCGuiWidget ui_;
  QWidget* widget_;
};
} // namespace

Q_DECLARE_METATYPE(std_msgs::StringConstPtr);
Q_DECLARE_METATYPE(geometry_msgs::PolygonStampedConstPtr);
Q_DECLARE_METATYPE(geometry_msgs::WrenchStampedConstPtr);
Q_DECLARE_METATYPE(std_msgs::Float32ConstPtr);
Q_DECLARE_METATYPE(sensor_msgs::ImuConstPtr);
Q_DECLARE_METATYPE(sensor_msgs::JointStateConstPtr);
Q_DECLARE_METATYPE(std_msgs::Int32MultiArrayConstPtr);
Q_DECLARE_METATYPE(rosgraph_msgs::ClockConstPtr);

#endif // talos_wbc_gui