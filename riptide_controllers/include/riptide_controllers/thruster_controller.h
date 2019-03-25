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
  // riptide_msgs::ThrusterResiduals residuals;

  // Vector3d pos_buoyancy;
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
  // std::vector<Vector3d> pos_buoyancy_eig;
  bool isBuoyant;
  // Vector3d Fb;
  double buoyancyCoeffs[3];

  // MatrixXd MomentCoeffs_eig;

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
  // void SetBuoyancyCoeffs();
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
      residual[i] = residual[i] / T(mass_inertia[i + 3]) - T(command[i + 3]);
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


// NOTE: It seems that ceres already tries to minimze all outputs as it solves.
// Hence, it seems unnecessary to add two more equations to create a
// SLE (system of linear eqns) composed of 8 equations and 8 unknowns)

/******************************* Tune Buoyancy ********************************/
// Purpose: Find the Center of Buoyancy (CoB)
// These equations ASSUME the vehicle is stationary in the water, attempting to
// reach a target orientation, but is unable to reach the said target because
// the moments due to buoyancy have not been factored into the angular eqns yet.
// The publised output will be the location of the CoB in relation to the CoM
// NOTE: Vehicle MUST be roughly stationary for output to make physical sense

// Tune Roll
// Thrusters contributing to a POSITIVE moment: sway_fwd, sway_aft, heave_port_fwd, heave_port_aft
// Thrusters contributting to a NEGATIVE moment: heave_stbd_fwd, heave_stbd_aft
// Buoyancy Y and Z components produce moments about x-axis

/*
struct tuneRoll
{
  template <typename T>
  bool operator()(const T *const pos_buoyancy_y, const T *const pos_buoyancy_z, T *residual) const
  {
    residual[0] = T(R_w2b.getRow(1).z()) * T(buoyancy) * (-pos_buoyancy_z[0]) +
                  T(R_w2b.getRow(2).z()) * T(buoyancy) * pos_buoyancy_y[0] +
                  T(sway_fwd) * T(-pos_sway_fwd.z) + T(sway_aft) * T(-pos_sway_aft.z) +
                  T(heave_port_fwd) * T(pos_heave_port_fwd.y) + T(heave_port_aft) * T(pos_heave_port_aft.y) +
                  T(heave_stbd_fwd) * T(pos_heave_stbd_fwd.y) + T(heave_stbd_aft) * T(pos_heave_stbd_aft.y) -
                  (T(ang_v.z()) * T(ang_v.y())) * (T(Izz) - T(Iyy));
    return true;
  }
};

// Tune Pitch
// Thrusters contributing to a POSITIVE moment: heave_port_aft, heave_stbd_aft
// Thrusters contributting to a NEGATIVE moment: surge_port_lo, surge_stbd_lo, heave_port_fwd, heave_stbd_fwd
// Buoyancy X and Z components produce moments about y-axis
struct tunePitch
{
  template <typename T>
  bool operator()(const T *const pos_buoyancy_x, const T *const pos_buoyancy_z, T *residual) const
  {
    residual[0] = T(R_w2b.getRow(0).z()) * T(buoyancy) * pos_buoyancy_z[0] +
                  T(R_w2b.getRow(2).z()) * T(buoyancy) * (-pos_buoyancy_x[0]) +
                  T(surge_port_lo) * T(pos_surge_port_lo.z) + T(surge_stbd_lo) * T(pos_surge_stbd_lo.z) +
                  T(heave_port_aft) * T(-pos_heave_port_aft.x) + T(heave_stbd_aft) * T(-pos_heave_stbd_aft.x) +
                  T(heave_port_fwd) * T(-pos_heave_port_fwd.x) + T(heave_stbd_fwd) * T(-pos_heave_stbd_fwd.x) -
                  (T(ang_v.x()) * T(ang_v.z())) * (T(Ixx) - T(Izz));
    return true;
  }
};

// Tune Yaw
// Thrusters contributing to a POSITIVE moment: surge_stbd_lo, sway_fwd
// Thrusters contributting to a NEGATIVE moment: surge_port_lo, sway_aft
// Buoyancy X and Y components produce moments about z-axis
struct tuneYaw
{
  template <typename T>
  bool operator()(const T *const pos_buoyancy_x, const T *const pos_buoyancy_y, T *residual) const
  {
    residual[0] = T(R_w2b.getRow(0).z()) * T(buoyancy) * (-pos_buoyancy_y[0]) +
                  T(R_w2b.getRow(1).z()) * T(buoyancy) * (pos_buoyancy_x[0]) +
                  T(surge_port_lo) * T(-pos_surge_port_lo.y) + T(surge_stbd_lo) * T(-pos_surge_stbd_lo.y) +
                  T(sway_fwd) * T(pos_sway_fwd.x) + T(sway_aft) * T(pos_sway_aft.x) -
                  (T(ang_v.y()) * T(ang_v.x())) * (T(Iyy) - T(Ixx));
    return true;
  }
};

*/
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
