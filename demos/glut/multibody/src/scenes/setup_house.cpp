//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_house.h"

#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>
#include <OpenTissue/core/math/math_random.h>

void setup_house(Data & data)
{
  real_type const friction = 0.25;
  real_type const restitution = 0.15;

  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(491);


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
  real_type D = 1.;
  real_type W2 = W/value_traits::two();
  real_type H2 = H/value_traits::two();
  real_type D2 = D/value_traits::two();
  data.m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(W2,H2,D2));
  OpenTissue::geometry::compute_box_mass_properties(data.m_small_box.ext(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  int X1,X2,Y;
  X1 = 5;
  X2 = 8;
  Y = 10;
  {
    for(int x=0;x<X1;++x)
    {
      for(int y=0;y<Y;++y)
      {
        r = vector3_type(x*W - W2, 0, 0.5 + y*H + H2);
        if(y%2)
          r(0) += W2;
        data.m_bodies[i].attach(&data.m_gravity);
        data.m_bodies[i].set_position(r);
        data.m_bodies[i].set_orientation(Q);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);
        ++i;
      }
    }
  }
  {
    for(int x=0;x<X1;++x)
    {
      for(int y=0;y<Y;++y)
      {
        vector3_type p = vector3_type(x*W - W2, 0, 0.5 + y*H + H2);
        if(y%2)
          p(0) += W2;
        data.m_bodies[i].attach(&data.m_gravity);
        R = OpenTissue::math::Rz(value_traits::pi());
        //        real_type offset = (W2-D2);
        r = R*p + vector3_type(X1*W - (W + W2), X2*W, 0);
        data.m_bodies[i].set_position(r);
        Q.Rz(value_traits::pi());
        data.m_bodies[i].set_orientation(Q);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);
        ++i;
      }
    }
  }
  {
    for(int x=0;x<X2;++x)
    {
      for(int y=0;y<Y;++y)
      {
        vector3_type p = vector3_type(x*W - W2, 0, 0.5 + y*H + H2);
        if(y%2)
          p(0) += W2;
        data.m_bodies[i].attach(&data.m_gravity);
        R = OpenTissue::math::Rz(3.0*value_traits::pi()/value_traits::two());
        real_type offset = (W2-D2);
        r = R*p + vector3_type(-offset - W2, X2*W - offset - W2, 0);
        data.m_bodies[i].set_position(r);
        Q.Rz(3.0*value_traits::pi()/value_traits::two());
        data.m_bodies[i].set_orientation(Q);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);
        ++i;
      }
    }
  }
  {
    for(int x=0;x<X2;++x)
    {
      for(int y=0;y<Y;++y)
      {
        vector3_type p = vector3_type(x*W - W2, 0, 0.5 + y*H + H2);
        if(y%2)
          p(0) += W2;
        data.m_bodies[i].attach(&data.m_gravity);
        R = OpenTissue::math::Rz(value_traits::pi()/value_traits::two());
        real_type offset = (W2-D2);
        r = R*p + vector3_type(X1*W - offset - W2,W2 + D2, 0);
        data.m_bodies[i].set_position(r);
        Q.Rz(value_traits::pi()/value_traits::two());
        data.m_bodies[i].set_orientation(Q);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);
        ++i;
      }
    }
  }
  {
    Q.identity();
    R = OpenTissue::math::diag(value_traits::one());
    for(int y=0;y<Y;++y)
    {
      for(int x=0;x<X1-y;++x)
      {
        real_type offset = y*W2 - (W/4.0);
        real_type offset2 = Y*H+0.5 + H2;
        r = vector3_type(x*W + offset, 0, offset2 + y*H);
        data.m_bodies[i].attach(&data.m_gravity);
        data.m_bodies[i].set_position(r);
        data.m_bodies[i].set_orientation(Q);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);
        ++i;
      }
    }
  }
  {
    Q.identity();
    R = OpenTissue::math::diag(value_traits::one());
    for(int y=0;y<Y;++y)
    {
      for(int x=0;x<X1-y;++x)
      {
        real_type offset = y*W2 - (W/4.0);
        real_type offset2 = Y*H+0.5 + H2;
        vector3_type p = vector3_type(x*W + offset, 0, offset2 + y*H);
        R = OpenTissue::math::Rz(value_traits::pi());
        r = R*p + vector3_type(X1*W - (W + W2), X2*W, 0);
        data.m_bodies[i].set_position(r);
        Q.Rz(value_traits::pi());
        data.m_bodies[i].set_orientation(Q);
        data.m_bodies[i].attach(&data.m_gravity);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);
        ++i;
      }
    }
  }
  density *= 5;
  data.m_sphere[0].radius(.5);
  OpenTissue::geometry::compute_sphere_mass_properties(data.m_sphere[0].radius(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  for(int j=0;j<200;++j)
  {
    vector3_type p;
    random(p,-X1*W,X1*W);
    p(0) *= 2;
    p(1) *= 2;
    p(0) -= 300;
    p(1) -= 300;
    p(2) += 50;
    data.m_bodies[i].set_position(p);
    data.m_bodies[i].set_velocity(vector3_type(50,50,0));
    data.m_bodies[i].set_orientation(Q);
    data.m_bodies[i].attach(&data.m_gravity);
    data.m_bodies[i].set_geometry(&data.m_sphere[0]);
    data.m_bodies[i].set_mass(mass);
    data.m_bodies[i].set_inertia_bf(I);
    data.m_configuration.add(&data.m_bodies[i]);
    ++i;
  }
  data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();

  default_material->set_friction_coefficient(friction);
  default_material->normal_restitution() = (restitution);
  data.m_configuration.set_material_library(data.m_library);
  
  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
}


