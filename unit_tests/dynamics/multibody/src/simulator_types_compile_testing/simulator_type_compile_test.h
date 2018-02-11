#ifndef SIMULATOR_TYPE_COMPILE_TEST
#define SIMULATOR_TYPE_COMPILE_TEST
//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/math/mbd_default_math_policy.h>
#include <OpenTissue/dynamics/mbd/mbd.h>

template<typename types>
class MyCollisionDetection
  : public OpenTissue::mbd::CollisionDetection<
  types
  , OpenTissue::mbd::SpatialHashing
  , OpenTissue::mbd::GeometryDispatcher
  , OpenTissue::mbd::SingleGroupAnalysis
  >
{};


template<typename mbd_types>
void simulator_interface_compile_test()
{
  typedef typename mbd_types::math_policy::real_type              real_type;
  typedef typename mbd_types::simulator_type                      simulator_type;
  typedef typename mbd_types::configuration_type                  configuration_type;

  simulator_type      simulator;
  configuration_type  configuration;

  real_type  time_step = simulator.time();
  simulator.run(time_step);
  simulator.init(configuration);

  simulator.get_configuration();
  simulator.get_stepper();
  simulator.get_sleepy();
  simulator.get_collision_detection();
}


template<typename math_types, template <typename> class stepper_type,template <typename> class simulator_type>
void simulator_type_compile_test()
{
  typedef typename OpenTissue::mbd::Types<
      math_types
    , OpenTissue::mbd::NoSleepyPolicy
    , stepper_type
    , MyCollisionDetection
    , simulator_type
  > types1;

  void (*ptr)() = 0;
  
  ptr = &(simulator_interface_compile_test<types1> );
}

// SIMULATOR_TYPE_COMPILE_TEST
#endif
