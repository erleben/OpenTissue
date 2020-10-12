//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_large_mass.h"

#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>
#include <OpenTissue/core/math/math_random.h>

void setup_large_mass(Data & data)
{
  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.clear(); //Make sure we got new rigid bodies allocated! Otherwise cached information may survive!
  data.m_bodies.resize(3);

  matrix3x3_type R;
  R = OpenTissue::math::diag(value_traits::one());

  data.m_big_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(10,10,.5));
  data.m_bodies[0].set_fixed(true);
  data.m_bodies[0].set_geometry(&data.m_big_box);
  data.m_configuration.add(&data.m_bodies[0]);

  real_type mass;
  vector3_type diag_inertia;
  matrix3x3_type I;

  real_type density = 10;
  data.m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(.5,.5,.5));
  OpenTissue::geometry::compute_box_mass_properties(data.m_small_box.ext(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));

  data.m_bodies[1].attach(&data.m_gravity);
  data.m_bodies[1].set_position(vector3_type(0,0,1.0));
  data.m_bodies[1].set_mass(mass);
  data.m_bodies[1].set_inertia_bf(I);
  data.m_bodies[1].set_geometry(&data.m_small_box);
  data.m_configuration.add(&data.m_bodies[1]);

  density = 1000;
  data.m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(1.5,1.5,1.5));
  OpenTissue::geometry::compute_box_mass_properties(data.m_box.ext(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));

  data.m_bodies[2].attach(&data.m_gravity);
  data.m_bodies[2].set_position(vector3_type(0,0,3.0));
  data.m_bodies[2].set_mass(mass);
  data.m_bodies[2].set_inertia_bf(I);
  data.m_bodies[2].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[2]);

  data.m_gravity.set_acceleration(vector3_type(value_traits::zero(),value_traits::zero(),-9.81));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();
  
  default_material->set_number_of_friction_directions(0);
  default_material->normal_restitution() = (value_traits::zero());
  
  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(1000);
  data.m_simulator.get_stepper()->warm_starting()      = true;
  data.m_simulator.get_stepper()->use_stabilization()  = true;
  data.m_simulator.get_stepper()->use_friction()       = true;
  data.m_simulator.get_stepper()->use_bounce()         = true;
}
