// Copyright 2018 Brazilian Intitute of Robotics"

#include <functional>
#include <boost/scoped_ptr.hpp>
#include <math.h>

// Gazebo Dependencies
#include "gazebo/rendering/ogre_gazebo.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>

// Ros Dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// MSISonar Dependencies
#include "mechanical_scanning_imaging_sonar_gazebo/MSISonar.hh"

namespace gazebo
{

enum RotAxis
{
  X = 0,
  Y = 1,
  Z = 2
};

class MSISonarRos : public SensorPlugin
{
public:
  /**
   * @brief Documentation Iherited
   *
   * @param _parent
   * @param _sdf
   */
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /**
   * @brief Called for pre render
   *
   */
  void OnPreRender();

  /**
   * @brief Called For Render
   *
   */
  void OnUpdate();

  /**
   * @brief Called for post render
   *
   */
  void OnPostRender();

  /**
   * @brief Called for pose updating
   *
   */
  void OnPoseUpdate();

public:
  //// \brief Scene parent containing sensor
  rendering::ScenePtr scene;

  //// \brief World that contatins the sensor
  physics::WorldPtr world;

  //// \brief Parent Entity of the sensor
  physics::EntityPtr parent;

  //// \brief Link that the sensor is attached to
  physics::LinkPtr current;

  //// \brief sonar sensor where the link link is attached to
  std::shared_ptr<gazebo::rendering::MSISonar> sonar;

protected:
  /// \brief Get angle from pose
  double GetAngleFromPose(math::Pose _pose);

private:
  // Pointer to the model
  sensors::SensorPtr sensor;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Pointer to the update event connection
  event::ConnectionPtr updatePostRender;

  // Pointer to the update event connection
  event::ConnectionPtr updatePreRender;

  // Pointer to the update event connection
  event::ConnectionPtr updatePose;

  // Ros node handle
  std::unique_ptr<ros::NodeHandle> rosNode;

  // Ros image transport for sonar image
  std::unique_ptr<image_transport::ImageTransport> sonarImageTransport;

  // Image transport publisher for sonar image
  image_transport::Publisher sonarImagePub;

  // Ros image transport for shader image
  std::unique_ptr<image_transport::ImageTransport> shaderImageTransport;

  // Image transport publisher for shader image
  image_transport::Publisher shaderImagePub;

  // Rotational axis 1 -X 2 -Y 3 -Z
  RotAxis rotationAxis;

  // Initial angle
  double angleInit;

  // Actual angle
  double angleAct;

  // Delta rotion
  double angDispl;

  // Maximum angle
  double angleMax;

  // Minimum angle
  double angleMin;

  // Delta angular Velocity
  double angularVelocity;

  // Sampling frequency
  double samplingFrequency;

  // Timer to check the update frequency
  common::Timer updateTimer;

  // Debug flag
  bool bDebug;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(MSISonarRos)
}  // namespace gazebo
