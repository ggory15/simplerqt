#include "talos_wbc_gui/talos_wbc_gui.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <iostream>
#include <vector>


namespace talos_wbc_gui {

MyQGraphicsScene::MyQGraphicsScene(QWidget *parent) : QGraphicsScene(parent)
{
}

MyQGraphicsView::MyQGraphicsView(QWidget *parent) : QGraphicsView(parent)
{
}

void MyQGraphicsScene::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    //std::cout << parent()->findChild<QObject *>("graphicsViewCustom")->objectName().toStdString() << std::endl;

    QGraphicsView *view_ = parent()->findChild<QGraphicsView *>("graphicsViewCustom");

    //view_->setViewport();

    double scaleFactor = 0.1;
    const qreal minFactor = 1.0;
    const qreal maxFactor = 10.0;
    static qreal h11 = 1.0;
    static qreal h22 = 1.0;

    if (event->delta() == 120)
    {
        h11 = (h11 >= maxFactor) ? h11 : (h11 + scaleFactor);
        h22 = (h22 >= maxFactor) ? h22 : (h22 + scaleFactor);
    }
    else if (event->delta() == -120)
    {
        h11 = (h11 <= minFactor) ? minFactor : (h11 - scaleFactor);
        h22 = (h22 <= minFactor) ? minFactor : (h22 - scaleFactor);
    }
    view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    view_->setTransform(QTransform(h11, 0, 0, 0, h22, 0, 0, 0, 1));
}

TalosWBCGui::TalosWBCGui()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  qRegisterMetaType<std_msgs::StringConstPtr>();
  qRegisterMetaType<geometry_msgs::PolygonStampedConstPtr>();
  qRegisterMetaType<std_msgs::Float32ConstPtr>();
  qRegisterMetaType<sensor_msgs::ImuConstPtr>();
  qRegisterMetaType<std_msgs::Int32MultiArrayConstPtr>();
  qRegisterMetaType<rosgraph_msgs::ClockConstPtr>();
  qRegisterMetaType<sensor_msgs::JointStateConstPtr>();
  qRegisterMetaType<geometry_msgs::WrenchStampedConstPtr>();
  setObjectName("TalosWBCGui");

  timesub = nh_.subscribe("/clock", 1, &TalosWBCGui::TimerCallback, this);
  jointsub = nh_.subscribe("/joint_states", 1, &TalosWBCGui::JointStateCallback, this);
  
  forcesubs.push_back(nh_.subscribe("/right_ankle_ft", 1, &TalosWBCGui::ForceSensorCallback, this));
  forcesubs.push_back(nh_.subscribe("/left_ankle_ft", 1, &TalosWBCGui::ForceSensorCallback, this));
  forcesubs.push_back(nh_.subscribe("/right_wrist_ft", 1, &TalosWBCGui::ForceSensorCallback, this));
  forcesubs.push_back(nh_.subscribe("/left_wrist_ft", 1, &TalosWBCGui::ForceSensorCallback, this));

  imusub = nh_.subscribe("/base_imu", 1, &TalosWBCGui::IMUSensorCallback, this);
}

void TalosWBCGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
      widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  // add widget to the user interface
  context.addWidget(widget_);

  std::string mode;
  nh_.param<std::string>("/gui/", mode, "default"); //TODO bool for simulation or real robot

  connect(ui_.stat_btn, SIGNAL(pressed()), this, SLOT(jstatebtn()));
  connect(ui_.stat_btn_2, SIGNAL(pressed()), this, SLOT(sstatebtn()));


  ui_.stat_btn->setShortcut(QKeySequence(Qt::Key_1));
  ui_.stat_btn_2->setShortcut(QKeySequence(Qt::Key_2));

  // for page 1
  connect(this, &TalosWBCGui::TimerCallback, this, &TalosWBCGui::timercb);
  connect(this, &TalosWBCGui::JointStateCallback, this, &TalosWBCGui::jointcb);
  
  // for page 2
  connect(this, &TalosWBCGui::ForceSensorCallback, this, &TalosWBCGui::forcecb);
  connect(this, &TalosWBCGui::IMUSensorCallback, this, &TalosWBCGui::imucb);
  
}

void TalosWBCGui::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void TalosWBCGui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void TalosWBCGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void TalosWBCGui::timercb(const rosgraph_msgs::ClockConstPtr &msg)
{
    robot_time = msg->clock;  
    ui_.currenttime->setText(QString::number(robot_time.sec + robot_time.nsec/1e9, 'f', 3));
}

void TalosWBCGui::jointcb(const sensor_msgs::JointStateConstPtr &msg)
{
  // Left Leg
  int index = 18;
  ui_.LL_p_0->setText(QString::number(msg->position[index], 'f', 3));
  ui_.LL_p_1->setText(QString::number(msg->position[index+1], 'f', 3));
  ui_.LL_p_2->setText(QString::number(msg->position[index+2], 'f', 3));
  ui_.LL_p_3->setText(QString::number(msg->position[index+3], 'f', 3));
  ui_.LL_p_4->setText(QString::number(msg->position[index+4], 'f', 3));
  ui_.LL_p_5->setText(QString::number(msg->position[index+5], 'f', 3));
  
  ui_.LL_v_0->setText(QString::number(msg->velocity[index], 'f', 3));
  ui_.LL_v_1->setText(QString::number(msg->velocity[index + 1], 'f', 3));
  ui_.LL_v_2->setText(QString::number(msg->velocity[index + 2], 'f', 3));
  ui_.LL_v_3->setText(QString::number(msg->velocity[index + 3], 'f', 3));
  ui_.LL_v_4->setText(QString::number(msg->velocity[index + 4], 'f', 3));
  ui_.LL_v_5->setText(QString::number(msg->velocity[index + 5], 'f', 3));

  ui_.LL_t_0->setText(QString::number(msg->effort[index], 'f', 3));
  ui_.LL_t_1->setText(QString::number(msg->effort[index + 1], 'f', 3));
  ui_.LL_t_2->setText(QString::number(msg->effort[index + 2], 'f', 3));
  ui_.LL_t_3->setText(QString::number(msg->effort[index + 3], 'f', 3));
  ui_.LL_t_4->setText(QString::number(msg->effort[index + 4], 'f', 3));
  ui_.LL_t_5->setText(QString::number(msg->effort[index + 5], 'f', 3));

  index = 24;
  ui_.RL_p_0->setText(QString::number(msg->position[index], 'f', 3));
  ui_.RL_p_1->setText(QString::number(msg->position[index+1], 'f', 3));
  ui_.RL_p_2->setText(QString::number(msg->position[index+2], 'f', 3));
  ui_.RL_p_3->setText(QString::number(msg->position[index+3], 'f', 3));
  ui_.RL_p_4->setText(QString::number(msg->position[index+4], 'f', 3));
  ui_.RL_p_5->setText(QString::number(msg->position[index+5], 'f', 3));
  
  ui_.RL_v_0->setText(QString::number(msg->velocity[index], 'f', 3));
  ui_.RL_v_1->setText(QString::number(msg->velocity[index + 1], 'f', 3));
  ui_.RL_v_2->setText(QString::number(msg->velocity[index + 2], 'f', 3));
  ui_.RL_v_3->setText(QString::number(msg->velocity[index + 3], 'f', 3));
  ui_.RL_v_4->setText(QString::number(msg->velocity[index + 4], 'f', 3));
  ui_.RL_v_5->setText(QString::number(msg->velocity[index + 5], 'f', 3));

  ui_.RL_t_0->setText(QString::number(msg->effort[index], 'f', 3));
  ui_.RL_t_1->setText(QString::number(msg->effort[index + 1], 'f', 3));
  ui_.RL_t_2->setText(QString::number(msg->effort[index + 2], 'f', 3));
  ui_.RL_t_3->setText(QString::number(msg->effort[index + 3], 'f', 3));
  ui_.RL_t_4->setText(QString::number(msg->effort[index + 4], 'f', 3));
  ui_.RL_t_5->setText(QString::number(msg->effort[index + 5], 'f', 3));

  index = 0;
  ui_.LA_p_0->setText(QString::number(msg->position[index], 'f', 3));
  ui_.LA_p_1->setText(QString::number(msg->position[index+1], 'f', 3));
  ui_.LA_p_2->setText(QString::number(msg->position[index+2], 'f', 3));
  ui_.LA_p_3->setText(QString::number(msg->position[index+3], 'f', 3));
  ui_.LA_p_4->setText(QString::number(msg->position[index+4], 'f', 3));
  ui_.LA_p_5->setText(QString::number(msg->position[index+5], 'f', 3));
  ui_.LA_p_6->setText(QString::number(msg->position[index+6], 'f', 3));
  
  ui_.LA_v_0->setText(QString::number(msg->velocity[index], 'f', 3));
  ui_.LA_v_1->setText(QString::number(msg->velocity[index + 1], 'f', 3));
  ui_.LA_v_2->setText(QString::number(msg->velocity[index + 2], 'f', 3));
  ui_.LA_v_3->setText(QString::number(msg->velocity[index + 3], 'f', 3));
  ui_.LA_v_4->setText(QString::number(msg->velocity[index + 4], 'f', 3));
  ui_.LA_v_5->setText(QString::number(msg->velocity[index + 5], 'f', 3));
  ui_.LA_v_6->setText(QString::number(msg->velocity[index + 6], 'f', 3));

  ui_.LA_t_0->setText(QString::number(msg->effort[index], 'f', 3));
  ui_.LA_t_1->setText(QString::number(msg->effort[index + 1], 'f', 3));
  ui_.LA_t_2->setText(QString::number(msg->effort[index + 2], 'f', 3));
  ui_.LA_t_3->setText(QString::number(msg->effort[index + 3], 'f', 3));
  ui_.LA_t_4->setText(QString::number(msg->effort[index + 4], 'f', 3));
  ui_.LA_t_5->setText(QString::number(msg->effort[index + 5], 'f', 3));
  ui_.LA_t_6->setText(QString::number(msg->effort[index + 6], 'f', 3));

  index = 7;
  ui_.RA_p_0->setText(QString::number(msg->position[index], 'f', 3));
  ui_.RA_p_1->setText(QString::number(msg->position[index+1], 'f', 3));
  ui_.RA_p_2->setText(QString::number(msg->position[index+2], 'f', 3));
  ui_.RA_p_3->setText(QString::number(msg->position[index+3], 'f', 3));
  ui_.RA_p_4->setText(QString::number(msg->position[index+4], 'f', 3));
  ui_.RA_p_5->setText(QString::number(msg->position[index+5], 'f', 3));
  ui_.RA_p_6->setText(QString::number(msg->position[index+6], 'f', 3));
  
  ui_.RA_v_0->setText(QString::number(msg->velocity[index], 'f', 3));
  ui_.RA_v_1->setText(QString::number(msg->velocity[index + 1], 'f', 3));
  ui_.RA_v_2->setText(QString::number(msg->velocity[index + 2], 'f', 3));
  ui_.RA_v_3->setText(QString::number(msg->velocity[index + 3], 'f', 3));
  ui_.RA_v_4->setText(QString::number(msg->velocity[index + 4], 'f', 3));
  ui_.RA_v_5->setText(QString::number(msg->velocity[index + 5], 'f', 3));
  ui_.RA_v_6->setText(QString::number(msg->velocity[index + 6], 'f', 3));

  ui_.RA_t_0->setText(QString::number(msg->effort[index], 'f', 3));
  ui_.RA_t_1->setText(QString::number(msg->effort[index + 1], 'f', 3));
  ui_.RA_t_2->setText(QString::number(msg->effort[index + 2], 'f', 3));
  ui_.RA_t_3->setText(QString::number(msg->effort[index + 3], 'f', 3));
  ui_.RA_t_4->setText(QString::number(msg->effort[index + 4], 'f', 3));
  ui_.RA_t_5->setText(QString::number(msg->effort[index + 5], 'f', 3));
  ui_.RA_t_6->setText(QString::number(msg->effort[index + 6], 'f', 3));

  index = 16;
  ui_.H_p_0->setText(QString::number(msg->position[index], 'f', 3));
  ui_.H_p_1->setText(QString::number(msg->position[index+1], 'f', 3));

  ui_.H_v_0->setText(QString::number(msg->velocity[index], 'f', 3));
  ui_.H_v_1->setText(QString::number(msg->velocity[index+1], 'f', 3));

  ui_.H_t_0->setText(QString::number(msg->effort[index], 'f', 3));
  ui_.H_t_1->setText(QString::number(msg->effort[index+1], 'f', 3));

  index = 30;
  ui_.T_p_0->setText(QString::number(msg->position[index], 'f', 3));
  ui_.T_p_1->setText(QString::number(msg->position[index+1], 'f', 3));

  ui_.T_v_0->setText(QString::number(msg->velocity[index], 'f', 3));
  ui_.T_v_1->setText(QString::number(msg->velocity[index+1], 'f', 3));

  ui_.T_t_0->setText(QString::number(msg->effort[index], 'f', 3));
  ui_.T_t_1->setText(QString::number(msg->effort[index+1], 'f', 3));

  index = 14;
  ui_.LG_p->setText(QString::number(msg->position[index], 'f', 3));
  ui_.RG_p->setText(QString::number(msg->position[index+1], 'f', 3));

}

void TalosWBCGui::forcecb(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    if (msg->header.frame_id == "leg_right_6_link"){
      ui_.RL_f_x->setText(QString::number(msg->wrench.force.x, 'f', 3));
      ui_.RL_f_y->setText(QString::number(msg->wrench.force.y, 'f', 3));
      ui_.RL_f_z->setText(QString::number(msg->wrench.force.z, 'f', 3));
      ui_.RL_t_x->setText(QString::number(msg->wrench.torque.x, 'f', 3));
      ui_.RL_t_y->setText(QString::number(msg->wrench.torque.y, 'f', 3));
      ui_.RL_t_z->setText(QString::number(msg->wrench.torque.z, 'f', 3));    
    }
    else if (msg->header.frame_id == "leg_left_6_link"){
      ui_.LL_f_x->setText(QString::number(msg->wrench.force.x, 'f', 3));
      ui_.LL_f_y->setText(QString::number(msg->wrench.force.y, 'f', 3));
      ui_.LL_f_z->setText(QString::number(msg->wrench.force.z, 'f', 3));
      ui_.LL_t_x->setText(QString::number(msg->wrench.torque.x, 'f', 3));
      ui_.LL_t_y->setText(QString::number(msg->wrench.torque.y, 'f', 3));
      ui_.LL_t_z->setText(QString::number(msg->wrench.torque.z, 'f', 3));    
    }
    else if (msg->header.frame_id == "wrist_left_ft_link"){
      ui_.LA_f_x->setText(QString::number(msg->wrench.force.x, 'f', 3));
      ui_.LA_f_y->setText(QString::number(msg->wrench.force.y, 'f', 3));
      ui_.LA_f_z->setText(QString::number(msg->wrench.force.z, 'f', 3));
      ui_.LA_t_x->setText(QString::number(msg->wrench.torque.x, 'f', 3));
      ui_.LA_t_y->setText(QString::number(msg->wrench.torque.y, 'f', 3));
      ui_.LA_t_z->setText(QString::number(msg->wrench.torque.z, 'f', 3));    
    }
    else if (msg->header.frame_id == "wrist_right_ft_link"){
      ui_.RA_f_x->setText(QString::number(msg->wrench.force.x, 'f', 3));
      ui_.RA_f_y->setText(QString::number(msg->wrench.force.y, 'f', 3));
      ui_.RA_f_z->setText(QString::number(msg->wrench.force.z, 'f', 3));
      ui_.RA_t_x->setText(QString::number(msg->wrench.torque.x, 'f', 3));
      ui_.RA_t_y->setText(QString::number(msg->wrench.torque.y, 'f', 3));
      ui_.RA_t_z->setText(QString::number(msg->wrench.torque.z, 'f', 3));    
    }
}

void TalosWBCGui::imucb(const sensor_msgs::ImuConstPtr &msg)
{
  imu_quat.w() = msg->orientation.w;
  imu_quat.x() = msg->orientation.x;
  imu_quat.y() = msg->orientation.y;
  imu_quat.z() = msg->orientation.z;

  imu_rpy = imu_quat.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
  ui_.IMU1->setText(QString::number(imu_rpy(0), 'f', 3));  
  ui_.IMU2->setText(QString::number(imu_rpy(1), 'f', 3));  
  ui_.IMU3->setText(QString::number(imu_rpy(2), 'f', 3));     
}

void TalosWBCGui::jstatebtn()
{
    ui_.stackedWidget->setCurrentIndex(0);
}
void TalosWBCGui::sstatebtn()
{
    ui_.stackedWidget->setCurrentIndex(1);
}

} // namespace
PLUGINLIB_EXPORT_CLASS(talos_wbc_gui::TalosWBCGui, rqt_gui_cpp::Plugin)
