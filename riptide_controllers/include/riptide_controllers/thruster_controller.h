#ifndef THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLER_H

#include <math.h>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <dynamic_reconfigure/server.h>
#include <riptide_controllers/VehiclePropertiesConfig.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Accel.h"
#include "riptide_msgs/Imu.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/ThrustStamped.h"
#include "riptide_msgs/ThrusterResiduals.h"

#include "yaml-cpp/yaml.h"
#include "eigen3/Eigen/Eigen"
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, Dynamic, Dynamic, RowMajor> RowMatrixXd;

#define PI 3.141592653
#define GRAVITY 9.81 //[m/s^2]
#define WATER_DENSITY 1000.0 //[kg/m^3]

class ThrusterController
{
 private:
  // Comms
  ros::NodeHandle nh;
  ros::Subscriber state_sub, depth_sub, buoyancy_sub;
  ros::Publisher buoyancy_pub, residual_pub;
  riptide_msgs::ThrustStamped thrust;
  riptide_msgs::ThrusterResiduals residuals;

  bool debug_controller; // If true, key params can be input via messages
  dynamic_reconfigure::Server<riptide_controllers::VehiclePropertiesConfig> server;
  dynamic_reconfigure::Server<riptide_controllers::VehiclePropertiesConfig>::CallbackType cb;

  // New publisher for the new EoM
  ros::Publisher NEOM;

  double Mass, Volume, Weight, Buoyancy, Jxx, Jyy, Jzz;
  YAML::Node Vehicle_Properties;
  std::vector<int> ThrustersEnabled; // To check if there are any thrusters down
  std::vector<Vector6d> ThrusterCoeffs;
  Vector6d weightLoad_eig;
  MatrixXd ThrusterCoeffs_eig;
  MatrixXd Thrusters;
  Vector3d COB;
  int numThrusters;

  double mass_inertia[6]; // First three are mass of vehicle, last three are intertia

  double weightLoad[6]; // Influences of weight and buoyancy force; 
                        // (mg - buyancy force) * sin()cos() for the first three
                        // buyancy force * distance for the last three

  double transportThm[6]; // First three are set to 0 (do not have the translational speed)
                          // Last three are -qr(Jzz - Jyy) -pr(Jxx - Jzz) -pq(Jyy - Jxx)

  double command[6]; // First three are translational acceleration
                     // Last three are angular translational acceleration (Both from imu)

  double forces[8]; // Results we want from using ceres
                    // Solved forces stored here

  // Ceres for the new problem of using eigen
  ceres::Problem NewProblem;
  ceres::Solver::Options NewOptions;
  ceres::Solver::Summary NewSummary;

  double pos_buoyancy[3]; // Solved center of buoyancy stored here
  bool isBuoyant;
  double buoyancyCoeffs[3];

  // Rotation Matrices: world to body, and body to world
  // Angular Velocity
  tf::Matrix3x3 R_w2b, R_b2w;
  tf::Vector3 euler_deg, euler_rpy, ang_v;          
  geometry_msgs::Vector3Stamped buoyancy_pos;
  double buoyancy_depth_thresh;

  // Locate Buoyancy EOMs
  ceres::Problem buoyancyProblem;
  ceres::Solver::Options buoyancyOptions;
  ceres::Solver::Summary buoyancySummary;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Have to include this one if fixed eigen vectors will be applied

  ThrusterController(char **argv);
  template <typename T>
  void LoadParam(std::string param, T &var);
  void LoadVehicleProperties();
  void SetThrusterCoeffs();
  void InitThrustMsg();
  void DynamicReconfigCallback(riptide_controllers::VehiclePropertiesConfig &config, uint32_t levels);
  void ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg);
  void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
  void AccelCB(const geometry_msgs::Accel::ConstPtr &a);
  void Loop();
};

class EOM {
  private:
  int *numThrusters;
  double *mass_inertia, *weightLoad, *transportThm, *command;
  MatrixXd ThrusterCoeffs;

  public:
  EOM(int* num, const Ref<const MatrixXd> &ThrusterCoeffsIn, double* inertiaIn, double* weightLoadIn, double* transportThmIn, double* commandIn)
  {
    numThrusters = num;
    mass_inertia = inertiaIn;
    weightLoad = weightLoadIn;
    transportThm = transportThmIn;
    command = commandIn;
    ThrusterCoeffs = ThrusterCoeffsIn;
  }

  template <typename T>
  bool operator()(const T *const forces, T* residual) const {
    for (int i = 0; i < 6; i++) {
      residual[i] = T(0);

      for (int j = 0; j < *numThrusters; j++) {
        residual[i] = residual[i] + T(ThrusterCoeffs(i,j)) * forces[j];
      }

      residual[i] = residual[i] + T(weightLoad[i]) + T(transportThm[i]);
      residual[i] = residual[i] / T(mass_inertia[i]) - T(command[i]);
    }
    return true;
  }
};

class tuneCOB {
  private:
  int *numThrusters;
  double *mass_inertia, *weightLoad, *transportThm, *command, *forces, *buoyancyCoeffs;
  MatrixXd Thrusters;
  Vector3d Fb;

  public:
  tuneCOB(int* num, const Ref<const MatrixXd> &ThrustersIn, double *inertiaIn, double* weightLoadIn, double* transportThmIn, double* commandIn, double* forcesIn, double* buoyancyCoeffsIn) {
    numThrusters = num;
    mass_inertia = inertiaIn;
    weightLoad = weightLoadIn;
    transportThm = transportThmIn;
    command = commandIn;
    forces = forcesIn;
    Thrusters = ThrustersIn;
    buoyancyCoeffs = buoyancyCoeffsIn;
  }

  template <typename T>
  bool operator()(const T *const pos_buoyancy, T* residual) const {
    for (int i = 0; i < 3; i ++) {
      residual[i] = T(0);
      for (int j = 0; j < *numThrusters; j++) {
        residual[i] = residual[i] + T(Thrusters(j, i)) * forces[j];
      }
      if (i == 0) {
        residual[i] = residual[i] + buoyancyCoeffs[1] * (-1) * pos_buoyancy[2] + buoyancyCoeffs[2] * pos_buoyancy[1];
      }
      else if (i == 1) {
        residual[i] = residual[i] + buoyancyCoeffs[0] * pos_buoyancy[2] + buoyancyCoeffs[2] * (-1) * pos_buoyancy[0];
      }
      else if (i == 2) {
        residual[i] = residual[i] + buoyancyCoeffs[0] * (-1) * pos_buoyancy[1] + buoyancyCoeffs[1] * pos_buoyancy[0];
      }
      residual[i] = residual[i] + T(transportThm[i + 3]);
    }
  }
};

// Thrust limits (N):
// These limits cannot be set too low b/c otherwise it will interfere with
// the EOMs and result in additional thrusters turning on to maintain those
// relationships. Ex. surge and sway will kick in and move the vehicle at a diagonal
// when the heave thrust is capped at too low of a number. If these limits are
// laxed, then the solver will NOT turn on those additional thrusters and the
// output will be as expected.
// NOTE: For the time being, the upper/lower bounds have been REMOVED from the solver
double MIN_THRUST = -24.0;
double MAX_THRUST = 24.0;


/************************** Reconfigure Active Thrusters **********************/
// These structs are used only if a thruster is down (problem with the copro,
// thruster itself, etc.). They will force ceres to set their thrust output to
// zero, forcing it to change how it uses the other active thrusters to provide
// the desired acceleration.

// Disable Surge Port Lo
struct disableSPL
{
  template <typename T>
  bool operator()(const T *const surge_port_lo, T *residual) const
  {
    residual[0] = surge_port_lo[0];
    return true;
  }
};

// Disable Surge Stbd Lo
struct disableSSL
{
  template <typename T>
  bool operator()(const T *const surge_stbd_lo, T *residual) const
  {
    residual[0] = surge_stbd_lo[0];
    return true;
  }
};

// Disable Sway Fwd
struct disableSWF
{
  template <typename T>
  bool operator()(const T *const sway_fwd, T *residual) const
  {
    residual[0] = sway_fwd[0];
    return true;
  }
};

// Disable Sway Aft
struct disableSWA
{
  template <typename T>
  bool operator()(const T *const sway_aft, T *residual) const
  {
    residual[0] = sway_aft[0];
    return true;
  }
};

// Disable Heave Port Fwd
struct disableHPF
{
  template <typename T>
  bool operator()(const T *const heave_port_fwd, T *residual) const
  {
    residual[0] = heave_port_fwd[0];
    return true;
  }
};

// Disable Heave Stbd Fwd
struct disableHSF
{
  template <typename T>
  bool operator()(const T *const heave_stbd_fwd, T *residual) const
  {
    residual[0] = heave_stbd_fwd[0];
    return true;
  }
};

// Disable Heave Port Aft
struct disableHPA
{
  template <typename T>
  bool operator()(const T *const heave_port_aft, T *residual) const
  {
    residual[0] = heave_port_aft[0];
    return true;
  }
};

// Disable Heave Stbd Aft
struct disableHSA
{
  template <typename T>
  bool operator()(const T *const heave_stbd_aft, T *residual) const
  {
    residual[0] = heave_stbd_aft[0];
    return true;
  }
};
///////////////////////////////////////////////////////////////////////////////

#endif
