// Copyright 2018 mccr_gazebo"

#include "mechanical_scanning_imaging_sonar_gazebo/MSISonarRos.hh"

#include "mechanical_scanning_imaging_sonar_gazebo/SDFTool.hh"

#include <sensor_msgs/Range.h>

#include "gazebo/common/Events.hh"


namespace gazebo
{

void MSISonarRos::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->sensor = _parent;

  this->updateConnection =  event::Events::ConnectRender(
                              std::bind(&MSISonarRos::OnUpdate, this));
  this->updatePostRender =  event::Events::ConnectPostRender(
                              std::bind(&MSISonarRos::OnPostRender, this));
  this->updatePreRender =  event::Events::ConnectPreRender(
                             std::bind(&MSISonarRos::OnPreRender, this));
  this->updatePose = event::Events::ConnectWorldUpdateEnd(
                             std::bind(&MSISonarRos::OnPoseUpdate, this));


  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create CameraSensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->sensor->WorldName();

  this->world = physics::get_world("default");
  parent = this->world->GetEntity(this->sensor->ParentName());

  GZ_ASSERT(parent, "This parent does not exisst");

  gzwarn << "link name :" << gazebo::SDFTool::GetSDFElement<std::string>(_sdf, "link_reference") << std::endl;

  current = parent->GetChildLink(gazebo::SDFTool::GetSDFElement<std::string>(_sdf, "link_reference"));
  GZ_ASSERT(current, "It must have this link");

  angleMax = gazebo::SDFTool::GetSDFElement<double>(_sdf, "angle_max");
  angleMin = gazebo::SDFTool::GetSDFElement<double>(_sdf, "angle_min");

  checkMax = true;

  rotationAxis = static_cast<RotAxis>(gazebo::SDFTool::GetSDFElement<int>(_sdf, "axis_rotation"));
  angleInit = this->GetAngleFromPose(current->GetRelativePose());
  angleAct = angleInit;

  this->localRotation = _sdf->Get<ignition::math::Vector3d>("local_rotation");

  samplingFrequency = gazebo::SDFTool::GetSDFElement<double>(_sdf, "update_rate");
  updateTimer.Start();

  angularVelocity = gazebo::SDFTool::GetSDFElement<double>(_sdf, "angular_velocity");

  this->scene = rendering::get_scene(worldName);

  // Get scene pointer
  gzwarn << rendering::RenderEngine::Instance()->SceneCount() << " Num Scenes" << std::endl;

  if (!this->scene)
  {
    this->scene = rendering::RenderEngine::Instance()->CreateScene(worldName, false, true);
  }

  if (scene != nullptr)
  {
    gzwarn << "Got Scene" << std::endl;
    double hfov = M_PI / 2;
    this->sonar = std::shared_ptr<rendering::MSISonar>(new rendering::MSISonar(this->sensor->Name(), this->scene, false));
    this->sonar->SetFarClip(100.0);
    this->sonar->Init();
    this->sonar->Load(_sdf);
    this->sonar->CreateTexture("GPUTexture");
  }

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }
  this->rosNode.reset(new ros::NodeHandle(""));

  this->sonarImageTransport.reset(new image_transport::ImageTransport(*this->rosNode));

  GZ_ASSERT(std::strcmp(_sdf->Get<std::string>("topic").c_str(), ""), "Topic name is not set");

  this->sonarImagePub = this->sonarImageTransport->advertise(_sdf->Get<std::string>("topic"), 1);

  this->bDebug = false;
  if (_sdf->HasElement("debug"))
  {
    this->bDebug = _sdf->Get<bool>("debug");
    if (this->bDebug)
    {
      this->shaderImageTransport.reset(new image_transport::ImageTransport(*this->rosNode));
      this->shaderImagePub = this->shaderImageTransport->advertise(_sdf->Get<std::string>("topic") + "/shader", 1);
    }
  }
}



void MSISonarRos::OnPreRender()
{
  math::Pose prePose = current->GetRelativePose();
  current->SetRelativePose( prePose + math::Pose(0,0,0,this->localRotation.X(),this->localRotation.Y(),this->localRotation.Z()));
  this->sonar->PreRender(current->GetWorldCoGPose());
  current->SetRelativePose(prePose);

  current->SetRelativePose( prePose + math::Pose(0,0,0,0,0,-angleInit));
  angDispl = this->GetAngleFromPose(current->GetRelativePose());
  current->SetRelativePose(prePose);
  angleAct = this->GetAngleFromPose(current->GetRelativePose());

  // gzwarn << "Angle Disp" << angDispl << "check max " << checkMax << std::endl;

  this->sonar->GetSonarImage(angDispl);
}

void MSISonarRos::OnPoseUpdate()
{
  if(angleMax==angleMin)
    angularVelocity = angularVelocity;
  else if((angleMax <= angDispl) && checkMax )
  {
      angularVelocity = -angularVelocity;
      checkMax = false;
  }
  else if((angleMin >= angDispl) && !checkMax )
  {
      angularVelocity = -angularVelocity;
      checkMax = true;
  }

  current->SetAngularVel(math::Vector3(0,0,angularVelocity));
}

void MSISonarRos::OnUpdate()
{
  if (this->bDebug)
    gzwarn << this->sensor->ParentName() << std::endl;

  this->sonar->RenderImpl();
}


void MSISonarRos::OnPostRender()
{
  if(updateTimer.GetElapsed().Double() > 1/samplingFrequency)
  {
    updateTimer.Start(); 
    updateTimer = common::Timer(); updateTimer.Start(); 
  }
  else
    return;
  
  this->sonar->PostRender();

  // Publish sonar image
  {
    cv::Mat sonarImage = this->sonar->SonarImage();
    cv::Mat sonarMask = this->sonar->SonarMask();

    cv::Mat B = cv::Mat::zeros(sonarImage.rows, sonarImage.cols, CV_8UC1);
    sonarImage.convertTo(B, CV_8UC1, 255);

    cv::applyColorMap(B, B, cv::COLORMAP_WINTER);
    B.setTo(0, ~sonarMask);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", B).toImageMsg();
    this->sonarImagePub.publish(msg);
  }

  // Publish shader image
  if (this->bDebug)
  {
    cv::Mat shaderImage = this->sonar->ShaderImage();
    cv::Mat B = cv::Mat::zeros(shaderImage.rows, shaderImage.cols, CV_8UC3);
    shaderImage.convertTo(B, CV_8UC3, 255);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", B).toImageMsg();
    this->shaderImagePub.publish(msg);
  }
}

double MSISonarRos::GetAngleFromPose(math::Pose _pose)
{
  switch(rotationAxis)
  {
    case RotAxis::X:
      return _pose.rot.GetRoll();
    case RotAxis::Y:
      return _pose.rot.GetPitch();
    case RotAxis::Z:
      return _pose.rot.GetYaw();
    default:
      gzerr << "This axis does not exists" << std::endl;
      break;
  }
}

}  // namespace gazebo



