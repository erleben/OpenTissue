//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/math/mbd_default_math_policy.h>
#include <OpenTissue/dynamics/mbd/mbd.h>

template<typename mbd_types>
void interface_compile_test()
{
  typename mbd_types::simulator_type                      simulator;
  typename mbd_types::math_policy::real_type  time_step = simulator.time();
  simulator.run(time_step);

  typename mbd_types::configuration_type                  configuration;
  simulator.init(configuration);

  typename mbd_types::material_library_type  material_library;
  configuration.set_material_library(material_library);

  typename mbd_types::body_type                            body;
  configuration.add(&body);
  configuration.remove(&body);

  typedef OpenTissue::mbd::Gravity<mbd_types>              gravity_type;
  gravity_type gravity;
  body.attach(&gravity);
  body.detach(&gravity);

  typedef OpenTissue::mbd::Damping<mbd_types>              damping_type;
  damping_type damping;
  body.attach(&damping);
  body.detach(&damping);

  typedef OpenTissue::mbd::SliderJoint<mbd_types>          slider_type;
  slider_type slider;
  configuration.add(&slider);
  configuration.remove(&slider);

  typedef OpenTissue::mbd::HingeJoint<mbd_types>           hinge_type;
  hinge_type hinge;
  configuration.add(&hinge);
  configuration.remove(&hinge);

  typedef OpenTissue::mbd::UniversalJoint<mbd_types>       universal_type;
  universal_type universal;
  configuration.add(&universal);
  configuration.remove(&universal);

  typedef OpenTissue::mbd::BallJoint<mbd_types>            ball_type;
  ball_type ball;
  configuration.add(&ball);
  configuration.remove(&ball);

  typedef OpenTissue::mbd::WheelJoint<mbd_types>           wheel_type;
  wheel_type wheel;
  configuration.add(&wheel);
  configuration.remove(&wheel);

  typedef OpenTissue::mbd::Oscillation<mbd_types>          oscillation_type;
  oscillation_type oscillation;
  body.set_scripted_motion(&oscillation);

  typedef OpenTissue::mbd::AngularJointMotor<mbd_types>    angular_motor_type;
  typedef OpenTissue::mbd::AngularJointLimit<mbd_types>    angular_limit_type;
  typedef OpenTissue::mbd::LinearJointMotor<mbd_types>     linear_motor_type;
  typedef OpenTissue::mbd::LinearJointLimit<mbd_types>     linear_limit_type;
  typedef OpenTissue::mbd::ReachCone<mbd_types>            reach_cone_type;
}

template<typename mbd_types>
void utilities_compile_test()
{
  typename mbd_types::simulator_type                      simulator;
  typename mbd_types::math_policy::real_type  time_step = simulator.time();
  simulator.run(time_step);

  typename mbd_types::configuration_type                  configuration;
  simulator.init(configuration);


  typename mbd_types::math_policy::vector_type energy;
  OpenTissue::mbd::compute_kinetic_energy_vector(configuration.body_begin(),configuration.body_end(),energy);

  typename mbd_types::math_policy::matrix_type C;
  OpenTissue::mbd::compute_contact_count_matrix(configuration.body_begin(),configuration.body_end(),C);

  OpenTissue::mbd::compute_contact_count(configuration.body_begin(),configuration.body_end());

  {
    float min_dist;
    float max_dist;
    OpenTissue::mbd::compute_min_max_distance(configuration, min_dist, max_dist);
  }

  {
    double min_dist;
    double max_dist;
    OpenTissue::mbd::compute_min_max_distance(configuration, min_dist, max_dist);
  }

  OpenTissue::mbd::mel::geometry_string(configuration.body_begin(),configuration.body_end());
  OpenTissue::mbd::mel::keyframe_string(configuration.body_begin(),configuration.body_end(),simulator.time());
  OpenTissue::mbd::mel::euler_filter_string(configuration.body_begin(),configuration.body_end());

  typename mbd_types::body_type * body = 0;


//   OpenTissue::mbd::draw_body(*body,true);
//
//   OpenTissue::mbd::DrawBodyFunctor<false>()(*body);
//   OpenTissue::mbd::DrawBodyFunctor<true>()(*body);
//
//   OpenTissue::mbd::draw_contacts(configuration);

  typename mbd_types::joint_type * joint = 0;


//   OpenTissue::mbd::draw_joint(*joint);
//   OpenTissue::mbd::DrawJointFunctor()(*joint);
//
//   OpenTissue::mbd::draw_penetrations(configuration);
}




template<typename types>
class MyCollisionDetection
  : public OpenTissue::mbd::CollisionDetection<
  types
  , OpenTissue::mbd::SpatialHashing
  , OpenTissue::mbd::GeometryDispatcher
  , OpenTissue::mbd::SingleGroupAnalysis
  >
{};


template< typename types  >
class stepper_type
  : public OpenTissue::mbd::ConstraintBasedShockPropagationStepper< types, OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy> >
{};


typedef OpenTissue::mbd::Types<
  OpenTissue::mbd::default_ublas_math_policy<float>
  , OpenTissue::mbd::NoSleepyPolicy
  , stepper_type
  , MyCollisionDetection
  , OpenTissue::mbd::ExplicitFixedStepSimulator
> types1;

void (*interface_ptr1)() = &(interface_compile_test<types1> );
void (*utility_ptr1)() = &(utilities_compile_test<types1> );


typedef OpenTissue::mbd::Types<
OpenTissue::mbd::default_ublas_math_policy<double>
, OpenTissue::mbd::NoSleepyPolicy
, stepper_type
, MyCollisionDetection
, OpenTissue::mbd::ExplicitFixedStepSimulator
> types2;

void (*interface_ptr2)() = &(interface_compile_test<types2> );
void (*utility_ptr2)() = &(utilities_compile_test<types2> );
