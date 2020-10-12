//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_card.h"

#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>
#include <OpenTissue/core/math/math_random.h>

void setup_card(Data & data)
{
  real_type const friction = 0.4;
  real_type const restitution = 0.15;

  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(43);


  matrix3x3_type R;
  R = OpenTissue::math::diag(value_traits::one());
  quaternion_type Q;
  Q.identity();
  vector3_type r;
  real_type mass;
  vector3_type diag_inertia;
  matrix3x3_type I;
  real_type density = 10;
  int i = 0;
  data.m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(800,800,.5));
  data.m_bodies[i].set_fixed(true);
  data.m_bodies[i].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;
  real_type W = 2.;
  real_type H = 1.;
  real_type D = .1;
  real_type W2 = W/value_traits::two();
  real_type H2 = H/value_traits::two();
  real_type D2 = D/value_traits::two();
  data.m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(W2,H2,D2));
  OpenTissue::geometry::compute_box_mass_properties(data.m_small_box.ext(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  //--- Tilted card poses
  real_type theta = value_traits::pi()/6.0;
  real_type theta_half = theta/value_traits::two();
  real_type phi = (value_traits::pi()-theta) / value_traits::two();
  real_type s1  = sin(theta_half);
  real_type s2  = sin( phi );
  real_type c1  = cos(theta_half);
  real_type c2  = cos( phi );
  real_type x1  = - W2*c2 - D2*c1;
  real_type x2  =   -x1;
  real_type y1  =   0;
  real_type y2  =   0;
  real_type z1  =   W2*s2 + D2*s1;
  real_type z2  =   z1;
  //  real_type width  = 2*x2;
  real_type height = 2*z1;

  int X = 5;
  int Y = 5;

  for(int y=0;y<Y;++y)
  {
    for(int x=0;x<X-y;++x)
    {
      data.m_bodies[i].set_position( vector3_type(x1  +  x*W  + y*W2 ,y1,z1 + .5   +y*(height+D)  ) );
      Q.Ry( -phi );
      data.m_bodies[i].set_orientation(Q);
      data.m_bodies[i].set_velocity( vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()) );
      data.m_bodies[i].set_spin( vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()) );
      data.m_bodies[i].attach(&data.m_gravity);
      data.m_bodies[i].set_geometry(&data.m_small_box);
      data.m_bodies[i].set_mass(mass);
      data.m_bodies[i].set_inertia_bf(I);
      data.m_configuration.add(&data.m_bodies[i]);
      ++i;
      data.m_bodies[i].set_position( vector3_type(x2 +  x*W + y*W2 ,y2,z2  +.5 +y*(height+D) ) );
      Q.Ry( phi );
      data.m_bodies[i].set_orientation(Q);
      data.m_bodies[i].set_velocity( vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()) );
      data.m_bodies[i].set_spin( vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()) );
      data.m_bodies[i].attach(&data.m_gravity);
      data.m_bodies[i].set_geometry(&data.m_small_box);
      data.m_bodies[i].set_mass(mass);
      data.m_bodies[i].set_inertia_bf(I);
      data.m_configuration.add(&data.m_bodies[i]);
      ++i;
      if(x>0)
      {
        data.m_bodies[i].set_position( vector3_type(x*W -W2 + y*W2, 0, height + D2 + .5  +y*(height+D)) );
        Q.identity();
        data.m_bodies[i].set_orientation(Q);
        data.m_bodies[i].set_velocity( vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()) );
        data.m_bodies[i].set_spin( vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()) );
        data.m_bodies[i].attach(&data.m_gravity);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);
        ++i;
      }
    }
  }

  data.m_sphere[0].radius(.5);
  OpenTissue::geometry::compute_sphere_mass_properties(data.m_sphere[0].radius(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  vector3_type p( 2,  -5, 3 + data.m_sphere[0].radius()+.5);
  data.m_bodies[i].set_position(p);
  data.m_bodies[i].set_velocity(vector3_type(0,5,0));
  data.m_bodies[i].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[i].set_orientation(Q);
  data.m_bodies[i].attach(&data.m_gravity);
  data.m_bodies[i].set_geometry(&data.m_sphere[0]);
  data.m_bodies[i].set_mass(mass);
  data.m_bodies[i].set_inertia_bf(I);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  p =vector3_type( 3.3,  7.5, 4 + data.m_sphere[0].radius()+.5);
  data.m_bodies[i].set_position(p);
  data.m_bodies[i].set_velocity(vector3_type(0,-6,0));
  data.m_bodies[i].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  data.m_bodies[i].set_orientation(Q);
  data.m_bodies[i].attach(&data.m_gravity);
  data.m_bodies[i].set_geometry(&data.m_sphere[0]);
  data.m_bodies[i].set_mass(mass);
  data.m_bodies[i].set_inertia_bf(I);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();
  
  default_material->set_friction_coefficient(friction);
  default_material->normal_restitution() = (restitution);
  
  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
}
