//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_silo.h"
#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>


void setup_silo(Data & data)
{
  real_type const  friction = 0.25;
  real_type const  restitution = 0.15;

  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(5000);

  matrix3x3_type R;
  R = OpenTissue::math::diag(value_traits::one());
  data.m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(15,15,.5));

  int i = 0;
  data.m_bodies[i].set_fixed(true);
  data.m_bodies[i].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  quaternion_type q;

  data.m_bodies[i].set_fixed(true);
  data.m_bodies[i].set_position(vector3_type(20,0,25));
  q.Ry(-5.0*value_traits::pi()/180.0);
  data.m_bodies[i].set_orientation(q);
  data.m_bodies[i].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_bodies[i].set_fixed(true);
  data.m_bodies[i].set_position(vector3_type(-20,0,25));
  q.Ry(5.0*value_traits::pi()/180.0);
  data.m_bodies[i].set_orientation(q);
  data.m_bodies[i].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_bodies[i].set_fixed(true);
  q.Rx(5.0*value_traits::pi()/180.0);
  data.m_bodies[i].set_orientation(q);
  data.m_bodies[i].set_position(vector3_type(0,20,25));
  data.m_bodies[i].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_bodies[i].set_fixed(true);
  q.Rx(-5.0*value_traits::pi()/180.0);
  data.m_bodies[i].set_orientation(q);
  data.m_bodies[i].set_position(vector3_type(0,-20,25));
  data.m_bodies[i].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  quaternion_type Q;
  vector3_type r;
  real_type mass;
  vector3_type diag_inertia;
  matrix3x3_type I;
  real_type density = 2000; //--- brick
  OpenTissue::geometry::compute_box_mass_properties(data.m_small_box.ext(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  real_type box_length = 1.5;
  real_type box_height = 1;
  data.m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(box_length/2.,box_height/2.,.5));
  size_type N = 32;
  size_type H = 5;
  real_type alpha = 2.*value_traits::pi()/N;
  real_type A = .5*(box_length + 0.01);
  real_type C = A / (std::sin(alpha/2.));
  real_type radius = std::sqrt(A*A + C*C) + .5;
  for(size_type n=0;n<N;++n)
  {
    for(size_type z=0;z<H;++z)
    {
      real_type angle = n*alpha;
      if(z%2)
        angle += alpha/2.;
      real_type x_val  = radius*std::cos(angle);
      real_type y_val  = radius*std::sin(angle);
      r = vector3_type(x_val, y_val, z*box_height + value_traits::one());
      Q.Rz((angle+value_traits::pi()/2.));
      data.m_bodies[i].set_position(r);
      data.m_bodies[i].set_orientation(Q);
      data.m_bodies[i].attach(&data.m_gravity);
      data.m_bodies[i].set_geometry(&data.m_small_box);
      data.m_bodies[i].set_inertia_bf(I);
      data.m_bodies[i].set_mass(mass);
      data.m_configuration.add(&data.m_bodies[i]);
      ++i;
    }
  }

  data.m_sphere[0].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),0.1);
  data.m_sphere[1].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),0.2);
  data.m_sphere[2].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),0.3);
  density = 120;  // wood thingy
  //while(i<10000)
  while(i<5000)
  {
    OpenTissue::geometry::compute_sphere_mass_properties(data.m_sphere[i%3].radius(),density,mass,diag_inertia);
    I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
    random(r,-12.0,12.0);
    r(2) *= 2;
    r(2) += 53;
    data.m_bodies[i].set_position(r);
    data.m_bodies[i].attach(&data.m_gravity);
    data.m_bodies[i].set_geometry(&data.m_sphere[i%3]);
    data.m_bodies[i].set_mass(mass);
    data.m_bodies[i].set_inertia_bf(I);
    data.m_configuration.add(&data.m_bodies[i]);
    ++i;
  }
  data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));

  material_type * default_material = data.m_library.default_material();

  default_material->set_friction_coefficient(friction);
  default_material->normal_restitution() = (restitution);

  data.m_simulator.init(data.m_configuration);

  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
}
