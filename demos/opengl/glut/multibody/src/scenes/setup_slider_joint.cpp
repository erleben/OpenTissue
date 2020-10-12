//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_slider_joint.h"

void setup_slider_joint(Data & data)
{
  real_type const friction    = 0.25;
  real_type const restitution = 0.15;
  real_type const timestep    = 0.01;

  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(3);

  quaternion_type Q;
  matrix3x3_type R;
  R = OpenTissue::math::diag(value_traits::one());

  data.m_boxes[0].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(30,1,30));
  data.m_boxes[1].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(2,.5,.5));

  data.m_bodies[0].set_position(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[0].set_orientation(quaternion_type(1,0,0,0));
  data.m_bodies[0].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[0].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[0].set_geometry(&data.m_boxes[0]);
  data.m_bodies[0].set_fixed(true);
  data.m_configuration.add(&data.m_bodies[0]);

  data.m_bodies[1].attach(&data.m_gravity);
  data.m_bodies[1].set_position(vector3_type(-2,5,0));
  data.m_bodies[1].set_orientation(quaternion_type(1,0,0,0));
  data.m_bodies[1].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[1].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[1].set_geometry(&data.m_boxes[1]);
  data.m_bodies[1].set_fixed(true);
  data.m_configuration.add(&data.m_bodies[1]);

  data.m_bodies[2].attach(&data.m_gravity);
  data.m_bodies[2].set_position(vector3_type(2,5,0));
  data.m_bodies[2].set_orientation(quaternion_type(1,0,0,0));
  data.m_bodies[2].set_velocity(vector3_type(0,5,5));
  data.m_bodies[2].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[2].set_geometry(&data.m_boxes[1]);
  data.m_bodies[2].set_fixed(false);
  data.m_configuration.add(&data.m_bodies[2]);

  data.m_socket_A.init(data.m_bodies[1],coordsys_type(vector3_type(2,0,0),quaternion_type()));
  data.m_socket_B.init(data.m_bodies[2],coordsys_type(vector3_type(-2,0,0),quaternion_type()));
  data.m_slider.connect(data.m_socket_A,data.m_socket_B);

  data.m_slider.set_frames_per_second(value_traits::one()/timestep);
  data.m_slider.set_error_reduction_parameter(0.8);

  data.m_linear_limit.set_frames_per_second(value_traits::one()/timestep);
  data.m_linear_limit.set_error_reduction_parameter(0.8);
  data.m_linear_limit.set_min_limit(-value_traits::one());
  data.m_linear_limit.set_max_limit( value_traits::one());
  data.m_slider.set_limit(data.m_linear_limit);

  data.m_linear_motor.set_frames_per_second(value_traits::one()/timestep);
  data.m_linear_motor.set_error_reduction_parameter(0.8);
  data.m_linear_motor.set_maximum_force(10.0);
  data.m_linear_motor.set_desired_speed(-value_traits::one());
  data.m_slider.set_motor(data.m_linear_motor);

  data.m_configuration.add(&data.m_slider);

  data.m_gravity.set_acceleration(vector3_type(0,-9.81,0));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();

  default_material->set_friction_coefficient(friction);
  default_material->normal_restitution() = (restitution);
  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
  data.m_simulator.get_stepper()->warm_starting()      = false;
  data.m_simulator.get_stepper()->use_stabilization()  = true;
  data.m_simulator.get_stepper()->use_friction()        = true;
  data.m_simulator.get_stepper()->use_bounce()         = true;
}


