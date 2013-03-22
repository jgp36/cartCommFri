#ifndef OROCOS_CARTCOMMFRI_COMPONENT_HPP
#define OROCOS_CARTCOMMFRI_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <rtt/os/TimeService.hpp>

using namespace RTT;

class CartCommFri : public RTT::TaskContext{
 
  //Input Port
  InputPort<geometry_msgs::Pose> port_cart_pos_;
  geometry_msgs::Pose cart_pos_;

  //Output Ports
  OutputPort<geometry_msgs::Wrench> port_cart_wrench_cmd_;
  geometry_msgs::Wrench cart_wrench_cmd_;
  OutputPort<lwr_fri::CartesianImpedance> port_cart_imp_cmd_;
  lwr_fri::CartesianImpedance cart_imp_cmd_;

  //Orocos Properties
  float kp_;
  float kd_;
  float t_out_;

  //Other
  std::vector<float> pos_des_;
  std::vector<float> cart_vel_;
  std::vector<float> last_pos_;
  RTT::os::TimeService::ticks t_start_;
  RTT::os::TimeService::Seconds t_cur_;
  RTT::os::TimeService::Seconds t_last_;
  RTT::os::TimeService::Seconds t_disp_;
  
  public:
    CartCommFri(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
