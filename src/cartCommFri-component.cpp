#include "cartCommFri-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

CartCommFri::CartCommFri(std::string const& name) : TaskContext(name){
  std::cout << "CartCommFri constructed !" <<std::endl;
}

bool CartCommFri::configureHook(){
  std::cout << "CartCommFri configured !" <<std::endl;
  return true;
}

bool CartCommFri::startHook(){
  std::cout << "CartCommFri started !" <<std::endl;
  return true;
}

void CartCommFri::updateHook(){
  std::cout << "CartCommFri executes updateHook !" <<std::endl;
}

void CartCommFri::stopHook() {
  std::cout << "CartCommFri executes stopping !" <<std::endl;
}

void CartCommFri::cleanupHook() {
  std::cout << "CartCommFri cleaning up !" <<std::endl;
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
