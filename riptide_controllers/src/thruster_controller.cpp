#include "riptide_controllers/thruster_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster_controller");
  ThrusterController ThrusterController(argv);
  ThrusterController.Loop();
}

ThrusterController::ThrusterController(char **argv) : nh("thruster_controller") {
  // Commands for the new control system
  // Load the vehicle properties yaml file 
  Vehicle_Properties = YAML::LoadFile("../osu-uwrt/riptide_software/src/riptide_controllers/cfg/vehicle_properties2.yaml");
  ThrusterController::LoadVehicleProperties();
  ThrusterController::SetThrusterCoeffs();
  weightLoad_eig.setZero();
  // buoyancyCoeffs.setZero();
  // Fb.setZero();

  for (int i = 0; i < 6; i++) {
    weightLoad[i] = 0;
    transportThm[i] = 0;
    command[i] = 0;
  } 

  nh.getParam("debug", debug_controller);
  ThrusterController::LoadParam("buoyancy_depth_thresh", buoyancy_depth_thresh); // Depth threshold to include buoyancy

  R_b2w.setIdentity();
  R_w2b.setIdentity();
  euler_rpy.setZero();
  ang_v.setZero();

  isBuoyant = false;

  state_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &ThrusterController::ImuCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &ThrusterController::DepthCB, this);
  // residual_pub = nh.advertise<riptide_msgs::ThrusterResiduals>("/status/controls/thruster", 1);
  
  // New publisher for using eigen
  NEOM = nh.advertise<riptide_msgs::ThrustStamped>("/command/thrust", 1);

  // Dynamic Reconfigure Variables
  cb = boost::bind(&ThrusterController::DynamicReconfigCallback, this, _1, _2);
  server.setCallback(cb);

  // Debug variables
  if(debug_controller) {
    buoyancy_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/debug/pos_buoyancy", 1);
    // Published in a message
    buoyancy_pos.vector.x = 0;
    buoyancy_pos.vector.y = 0;
    buoyancy_pos.vector.z = 0;   
  }

  ThrusterController::InitThrustMsg();

  google::InitGoogleLogging(argv[0]);

  // PROBLEM SETUP

  /////////////////////////////////////////////////////////////////////////////
  // NEW PROBLEM
  NewProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<EOM, 6, 8>(new EOM(&numThrusters, ThrusterCoeffs_eig, mass_inertia, weightLoad, transportThm, command)), NULL, forces);
  NewOptions.max_num_iterations = 100;
  NewOptions.linear_solver_type = ceres::DENSE_QR;

  /////////////////////////////////////////////////////////////////////////////
  // NEW BUOYANCY PROBLEM
  buoyancyProblem.AddResidualBlock(new ceres::AutoDiffCostFunction<tuneCOB, 1, 3>(new tuneCOB(&numThrusters, Thrusters, mass_inertia, weightLoad, transportThm, command, forces, buoyancyCoeffs)), NULL, pos_buoyancy);
  buoyancyOptions.max_num_iterations = 50;
  buoyancyOptions.linear_solver_type = ceres::DENSE_QR;
  
#ifdef progress
  options.minimizer_progress_to_stdout = true;
#endif
}

// Load parameter from namespace
template <typename T>
void ThrusterController::LoadParam(std::string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    std::string ns = nh.getNamespace();
    ROS_ERROR("Thruster Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void ThrusterController::LoadVehicleProperties() {
  Mass = Vehicle_Properties["Properties"]["Mass"].as<double>();
  Volume = Vehicle_Properties["Properties"]["Volume"].as<double>();

  Jxx = Vehicle_Properties["Properties"]["Inertia"][0].as<double>();
  Jyy = Vehicle_Properties["Properties"]["Inertia"][1].as<double>();
  Jzz = Vehicle_Properties["Properties"]["Inertia"][2].as<double>();
  
  for (int i = 0; i < 3; i++) {
    COB[i] = Vehicle_Properties["Properties"]["COB"][i].as<double>();
  }
  
  mass_inertia[0] = Mass;
  mass_inertia[1] = Mass;
  mass_inertia[2] = Mass;
  mass_inertia[3] = Jxx;
  mass_inertia[4] = Jyy;
  mass_inertia[5] = Jzz;
}

void ThrusterController::SetThrusterCoeffs() {
  numThrusters = 0;
  // numThrusters = Vehicle_Properties["Properties"]["Thrusters"].size();
  for (int i = 0; i < 8; i ++) {
    bool Thruster_enabled = Vehicle_Properties["Properties"]["Thrusters"][i]["enabled"].as<bool>();
    if (Thruster_enabled) {
      numThrusters++;
    }
    ThrustersEnabled.push_back((int)Thruster_enabled);
  }

  // Resize the thrusters vector
  // Row: five values in position vector
  // Col: number of thrusters that are still working
  Thrusters.resize(5, numThrusters);
  Thrusters.setZero();

  // Resize the thruster coefficients
  // Row: 6 DOF
  // Col: number of thrusters that are still working
  ThrusterCoeffs_eig.resize(6, numThrusters);
  ThrusterCoeffs_eig.setZero();

  for (int i = 0; i < numThrusters; i ++) {
    if (ThrustersEnabled[i] == 1) {
      for (int j = 0; j < 5; j ++) {
        Thrusters(j ,i) = Vehicle_Properties["Properties"]["Thrusters"][i]["locations"][j].as<double>();
      }
    }
  }

  for (int i = 0; i < numThrusters; i ++) {
    if (ThrustersEnabled[i] == 1) {
      float psi = Thrusters(3, i) * PI / 180;
      float theta = Thrusters(4, i) * PI / 180;
      ThrusterCoeffs_eig(0, i) = cos(psi) * cos(theta);
      ThrusterCoeffs_eig(1, i) = sin(psi) * cos(theta);
      ThrusterCoeffs_eig(2, i) = (-1) * sin(theta);

      ThrusterCoeffs_eig.block<3, 1>(3, i) = Thrusters.block<3, 1>(0, i).cross(ThrusterCoeffs_eig.block<3, 1>(0, i));
    }
  }
}

void ThrusterController::InitThrustMsg()
{
  riptide_msgs::ThrustStamped thrust;
  thrust.header.stamp = ros::Time::now();
  thrust.force.surge_port_lo = 0;
  thrust.force.surge_stbd_lo = 0;
  thrust.force.sway_fwd = 0;
  thrust.force.sway_aft = 0;
  thrust.force.heave_port_aft = 0;
  thrust.force.heave_stbd_aft = 0;
  thrust.force.heave_stbd_fwd = 0;
  thrust.force.heave_port_fwd = 0;
  NEOM.publish(thrust);
}

// Callback for dynamic reconfigure
void ThrusterController::DynamicReconfigCallback(riptide_controllers::VehiclePropertiesConfig &config, uint32_t levels) {
  if(debug_controller) {
    Mass = config.Mass;
    Volume = config.Volume;
    Weight = Mass * GRAVITY; 
    Buoyancy = Volume * WATER_DENSITY * GRAVITY;
  }
}

//Get orientation from IMU
void ThrusterController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  //Get euler angles in radians and make two rotation matrices
  vector3MsgToTF(imu_msg->rpy_rad, euler_rpy);
  R_b2w.setRPY(euler_rpy.x(), euler_rpy.y(), euler_rpy.z()); //Body to world rotations --> world_vector =  R_b2w * body_vector
  R_w2b = R_b2w.transpose(); //World to body rotations --> body_vector = R_w2b * world_vector

  //Get angular velocity and convert to [rad/s]
  vector3MsgToTF(imu_msg->ang_vel, ang_v);
  ang_v.setValue(ang_v.x()*PI/180, ang_v.y()*PI/180, ang_v.y()*PI/180);

  // Data for the new EoM
  Weight = Mass * GRAVITY;
  Buoyancy = Volume * GRAVITY * WATER_DENSITY;

  float phi = imu_msg->euler_rpy.x * PI / 180;
  float theta = imu_msg->euler_rpy.y * PI / 180;
  float p = imu_msg->ang_vel.x;
  float q = imu_msg->ang_vel.y;
  float r = imu_msg->ang_vel.z;

  transportThm[0] = 0;
  transportThm[1] = 0;
  transportThm[2] = 0; // The translational velocity is unknown, should be m * (r*v - q*w)
  transportThm[3] = (-1) * q * r * (Jzz - Jyy);
  transportThm[4] = (-1) * p * r * (Jxx - Jzz);
  transportThm[5] = (-1) * p * q * (Jyy - Jxx);

  weightLoad_eig(0) = (-1) * (Weight - Buoyancy) * sin(theta);
  weightLoad_eig(1) = (Weight - Buoyancy) * sin(phi) * cos(theta);
  weightLoad_eig(2) = (Weight - Buoyancy) * cos(phi) * cos(theta);

  Vector3d Fb;
  Fb(0) =  Buoyancy * sin(theta);
  Fb(1) = -Buoyancy * sin(phi) * cos(theta);
  Fb(2) = -Buoyancy * cos(phi) * cos(theta);

  weightLoad_eig.segment<3>(3) = COB.cross(Fb); 
  weightLoad_eig = weightLoad_eig * ((int)(isBuoyant));
  Map<RowMatrixXd>(&weightLoad[0], weightLoad_eig.rows(), weightLoad_eig.cols()) = weightLoad_eig;
  
  buoyancyCoeffs[0] = R_w2b.getRow(0).z() * Buoyancy;
  buoyancyCoeffs[1] = R_w2b.getRow(1).z() * Buoyancy;
  buoyancyCoeffs[2] = R_w2b.getRow(2).z() * Buoyancy;
}

//Get depth and determine if buoyancy should be included
void ThrusterController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg)
{
  if(depth_msg->depth > buoyancy_depth_thresh){
    isBuoyant = true;

  }
  else {
    isBuoyant = false;
  }
}

void ThrusterController::AccelCB(const geometry_msgs::Accel::ConstPtr &a) {
  // New command for new EoM
  command[0] = a->linear.x;
  command[1] = a->linear.y;
  command[2] = a->linear.z;
  command[3] = a->angular.x;
  command[4] = a->angular.y;
  command[5] = a->angular.z;

  // Ceres solver for solving foreces for the new EoM
  // Publish to the new publisher
  for (int i = 0; i < numThrusters; i ++) {
    forces[i] = 0; 
  }
  
  ceres::Solve(NewOptions, &NewProblem, &NewSummary);
  thrust.header.stamp = ros::Time::now();
  thrust.force.heave_port_fwd= -forces[0];
  thrust.force.heave_port_aft = -forces[1];
  thrust.force.heave_stbd_fwd = -forces[2];
  thrust.force.heave_stbd_aft = -forces[3];
  thrust.force.sway_fwd = -forces[4];
  thrust.force.sway_aft = -forces[5];
  thrust.force.surge_port_lo = -forces[6];
  thrust.force.surge_stbd_lo = -forces[7];
  NEOM.publish(thrust);

  if(debug_controller) {
  // Initialize values
    for (int i = 0; i < 3; i++) {
      pos_buoyancy[i] = 0;
    }

    ceres::Solve(buoyancyOptions, &buoyancyProblem, &buoyancySummary);
    buoyancy_pos.header.stamp = ros::Time::now();
    buoyancy_pos.vector.x = pos_buoyancy[0];
    buoyancy_pos.vector.y = pos_buoyancy[1];
    buoyancy_pos.vector.z = pos_buoyancy[2];
    buoyancy_pub.publish(buoyancy_pos);
  }
}

void ThrusterController::Loop()
{
  ros::Rate rate(200);
  while(!ros::isShuttingDown()) {
    ros::spinOnce();
    rate.sleep();
  }
}
