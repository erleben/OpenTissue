//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_domino_spiral.h"

#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>
#include <OpenTissue/core/math/math_random.h>

void setup_domino_spiral(Data & data)
{
  real_type const friction = .25;
  real_type const restitution = .4;

  real_type density = 10,mass;
  vector3_type r,diag_inertia;
  matrix3x3_type R,I;
  R = OpenTissue::math::diag(value_traits::one());
  quaternion_type Q;

  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();
  
  data.m_bodies.resize(502);

  data.m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(700,700,.5));
  data.m_bodies[0].set_fixed(true);
  data.m_bodies[0].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[0]);

  int i = 1;
  real_type W = .8;
  real_type H = .2;
  real_type D = value_traits::two();
  real_type W2 = .4;
  real_type H2 = .1;
  real_type D2 = value_traits::one();

  size_type N = 500;
  real_type radius = 3*W;
  assert(radius>=W);
  real_type theta = 0;

  data.m_sphere[0].radius(.25);
  OpenTissue::geometry::compute_sphere_mass_properties(data.m_sphere[0].radius(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  real_type x = radius*cos(theta-0.05);
  real_type y = radius*sin(theta-0.05);
  real_type z = 3.0*D;
  Q.Rz(theta);
  data.m_bodies[i].set_position(vector3_type(x,y,z));
  data.m_bodies[i].set_orientation(Q);
  data.m_bodies[i].attach(&data.m_gravity);
  data.m_bodies[i].set_geometry(&data.m_sphere[0]);
  data.m_bodies[i].set_mass(mass);
  data.m_bodies[i].set_inertia_bf(I);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(W2,H2,D2));
  OpenTissue::geometry::compute_box_mass_properties(data.m_small_box.ext(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));

  for(size_type n=0;n<N;++n)
  {
    x = radius*cos(theta);
    y = radius*sin(theta);
    z = D2 + .5;
    Q.Rz(theta);
    data.m_bodies[i].set_position(vector3_type(x,y,z));
    data.m_bodies[i].set_orientation(Q);
    data.m_bodies[i].attach(&data.m_gravity);
    data.m_bodies[i].set_geometry(&data.m_small_box);
    data.m_bodies[i].set_mass(mass);
    data.m_bodies[i].set_inertia_bf(I);
    data.m_configuration.add(&data.m_bodies[i]);

    real_type delta_theta = 0;
    real_type D4 = D*.25;
    if ((D4 +H)<=(value_traits::two()*radius))
      delta_theta = value_traits::two()* asin((D4+H)/(value_traits::two()*radius));
    else
      delta_theta = value_traits::two()*asin(H/radius);
    theta += delta_theta;
    radius = theta + 3*W;
    ++i;
  }

  //data.m_sphere[1].radius(.75);
  //compute_sphere_mass_properties(data.m_sphere[1].radius(),density,mass,diag_inertia);
  //I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  //x = 600;
  //y = 0;
  //z = data.m_sphere[1].radius() + .5;
  //Q.identity();
  //data.m_bodies[i].set_position(vector3_type(x,y,z));
  //data.m_bodies[i].set_velocity(vector3_type(-20,0,0));
  //data.m_bodies[i].set_orientation(Q);
  //data.m_bodies[i].attach(&data.m_gravity);
  //data.m_bodies[i].set_geometry(&data.m_sphere[1]);
  //data.m_bodies[i].set_mass(mass);
  //data.m_bodies[i].set_inertia_bf(I);
  //data.m_configuration.add(&data.m_bodies[i]);
  //++i;

  data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();

  default_material->set_friction_coefficient(friction);
  default_material->normal_restitution() = (restitution);
  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
}


