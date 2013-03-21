#ifndef OROCOS_CARTCOMMFRI_COMPONENT_HPP
#define OROCOS_CARTCOMMFRI_COMPONENT_HPP

#include <rtt/RTT.hpp>

class CartCommFri : public RTT::TaskContext{
  public:
    CartCommFri(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
