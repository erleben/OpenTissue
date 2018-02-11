//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_pyramid.h"

#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>
#include <OpenTissue/core/math/math_random.h>

void setup_pyramid(Data & data)
{
  real_type const friction = 0.25;
  real_type const restitution = 0.4;

  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(1245);


  matrix3x3_type R;
  R = OpenTissue::math::diag(value_traits::one());
  data.m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(500,500,.5));

  data.m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(.5,.5,.5));
  data.m_bodies[0].set_fixed(true);
  data.m_bodies[0].set_geometry(&data.m_box);
  data.m_configuration.add(&data.m_bodies[0]);
  quaternion_type Q;
  vector3_type r;
  real_type mass;
  vector3_type diag_inertia;
  matrix3x3_type I;
  real_type density = 10;
  OpenTissue::geometry::compute_box_mass_properties(data.m_small_box.ext(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));


  int height = 15;
  int i = 1;

  real_type centering =  height/2;

  for(int depth=height;depth>0;--depth)
  {
    int offset = height - depth;
    for(int x=1;x<=depth;++x)
    {
      for(int y=1;y<=depth;++y)
      {
        r = vector3_type(x+(.5*offset) - centering,y+(.5*offset) - centering,offset+value_traits::one());
        data.m_bodies[i].attach(&data.m_gravity);
        data.m_bodies[i].set_position(r);
        data.m_bodies[i].set_geometry(&data.m_small_box);
        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        data.m_configuration.add(&data.m_bodies[i]);

        ++i;
      }
    }
  }

  density *= 100;
  data.m_sphere[0].radius(.5);
  OpenTissue::geometry::compute_sphere_mass_properties(data.m_sphere[0].radius(),density,mass,diag_inertia);
  I= OpenTissue::math::diag(diag_inertia(0),diag_inertia(1),diag_inertia(2));
  data.m_bodies[i].set_position(vector3_type(-50,0,10));
  data.m_bodies[i].set_velocity(vector3_type(50,0,0));
  data.m_bodies[i].set_orientation(Q);
  data.m_bodies[i].attach(&data.m_gravity);
  data.m_bodies[i].set_geometry(&data.m_sphere[0]);
  data.m_bodies[i].set_mass(mass);
  data.m_bodies[i].set_inertia_bf(I);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_bodies[i].set_position(vector3_type(0,-100,10));
  data.m_bodies[i].set_velocity(vector3_type(0,50,0));
  data.m_bodies[i].set_orientation(Q);
  data.m_bodies[i].attach(&data.m_gravity);
  data.m_bodies[i].set_geometry(&data.m_sphere[0]);
  data.m_bodies[i].set_mass(mass);
  data.m_bodies[i].set_inertia_bf(I);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_bodies[i].set_position(vector3_type(-150,0,10));
  data.m_bodies[i].set_velocity(vector3_type(50,0,0));
  data.m_bodies[i].set_orientation(Q);
  data.m_bodies[i].attach(&data.m_gravity);
  data.m_bodies[i].set_geometry(&data.m_sphere[0]);
  data.m_bodies[i].set_mass(mass);
  data.m_bodies[i].set_inertia_bf(I);
  data.m_configuration.add(&data.m_bodies[i]);
  ++i;

  data.m_bodies[i].set_position(vector3_type(0,-200,10));
  data.m_bodies[i].set_velocity(vector3_type(0,50,0));
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

