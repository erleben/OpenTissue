//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_box.h"

void setup_box(Data & data)
{
  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(2);

  quaternion_type Q;
  matrix3x3_type R;
  R = OpenTissue::math::diag(value_traits::one());

  data.m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(10,10,1));
  data.m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(1,1,1));

  data.m_bodies[0].set_position(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[0].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[0].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[0].set_geometry(&data.m_box);
  data.m_bodies[0].set_fixed(true);
  data.m_configuration.add(&data.m_bodies[0]);

  data.m_bodies[1].attach(&data.m_gravity);
  data.m_bodies[1].set_position(vector3_type(value_traits::zero(),0,2.5));
  data.m_bodies[1].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[1].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[1].set_geometry(&data.m_small_box);
  data.m_configuration.add(&data.m_bodies[1]);

  data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();

  default_material->set_friction_coefficient(0.4);
  default_material->normal_restitution() = 0.1;

  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
  //data.m_simulator.get_stepper()->warm_starting()      = false;
  data.m_simulator.get_stepper()->warm_starting()      = true;
  data.m_simulator.get_stepper()->use_stabilization()  = true;
  data.m_simulator.get_stepper()->use_friction()        = true;
  data.m_simulator.get_stepper()->use_bounce()         = true;
}
