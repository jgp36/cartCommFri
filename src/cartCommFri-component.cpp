#include "cartCommFri-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

CartCommFri::CartCommFri(std::string const& name) : TaskContext(name){
  //Setup data ports
  this->addPort("CartesianPosition",port_cart_pos_);

  this->addPort("CartesianWrenchCommand", port_cart_wrench_cmd_);
  this->addPort("CarteisanImpedenceCommand", port_cart_imp_cmd_);

  //Setup properties
  this->addProperty("kp",kp_);
}

bool CartCommFri::configureHook(){
  //Set Cartesian Impedance - this overrides their position control-uggg x,y,z
  cart_imp_cmd_.stiffness.linear.x = 0;
  cart_imp_cmd_.stiffness.linear.y = 0;
  cart_imp_cmd_.stiffness.linear.z = 0;
  cart_imp_cmd_.stiffness.angular.x = 0;
  cart_imp_cmd_.stiffness.angular.y = 0;
  cart_imp_cmd_.stiffness.angular.z = 0;
  cart_imp_cmd_.damping.linear.x = 0;
  cart_imp_cmd_.damping.linear.y = 0;
  cart_imp_cmd_.damping.linear.z = 0;
  cart_imp_cmd_.damping.angular.x = 0;
  cart_imp_cmd_.damping.angular.y = 0;
  cart_imp_cmd_.damping.angular.z = 0;
  port_cart_imp_cmd_.write(cart_imp_cmd_);

  //Initialize outputs
  cart_wrench_cmd_.force.x = 0;
  cart_wrench_cmd_.force.y = 0;
  cart_wrench_cmd_.force.z = 0;
  cart_wrench_cmd_.torque.x = 0;
  cart_wrench_cmd_.torque.y = 0;
  cart_wrench_cmd_.torque.z = 0;
  port_cart_wrench_cmd_.write(cart_wrench_cmd_);

  //Set desired position to the current
  port_cart_pos_.read(cart_pos_);
  pos_des_.push_back(cart_pos_.position.x);
  pos_des_.push_back(cart_pos_.position.y);
  pos_des_.push_back(cart_pos_.position.z);


  return true;
}

bool CartCommFri::startHook(){

  return true;
}

void CartCommFri::updateHook(){

  //Read data
  if(port_cart_pos_.read(cart_pos_) == NewData) {
    
    //Position control - uggg pose is x,y,z
    cart_wrench_cmd_.force.x = kp_*(pos_des_[0] - cart_pos_.position.x);
    cart_wrench_cmd_.force.y = kp_*(pos_des_[1] - cart_pos_.position.y);
    cart_wrench_cmd_.force.z = kp_*(pos_des_[2] - cart_pos_.position.z);
    //Not bothering controlling using quaternions
    cart_wrench_cmd_.torque.x = 0;
    cart_wrench_cmd_.torque.y = 0;
    cart_wrench_cmd_.torque.z = 0;

  }
  port_cart_wrench_cmd_.write(cart_wrench_cmd_);
 
}

void CartCommFri::stopHook() {

}

void CartCommFri::cleanupHook() {
 
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(CartCommFri)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(CartCommFri)
