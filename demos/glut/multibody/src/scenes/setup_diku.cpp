//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_diku.h"

#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>
#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/utility/utility_get_environment_variable.h>

void setup_diku(Data & data)
{
  real_type const edge_resolution = 1.01;
  bool      const face_sampling = true;

  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(1001);

  data.m_sphere[0].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),0.165);
  data.m_sphere[1].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),2.*0.165);
  data.m_sphere[2].set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),3*0.165);

  if(data.m_diku_mesh.size_vertices()==0)
  {
    std::string data_path = OpenTissue::utility::get_environment_variable("OPENTISSUE");
    OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/diku.obj",data.m_diku_mesh);
    OpenTissue::mesh::scale(data.m_diku_mesh, 40.0, 40.0, 40.0);
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(data.m_diku_mesh);
    OpenTissue::sdf::semiauto_init_geometry(data.m_diku_mesh, edge_resolution, face_sampling, data.m_diku_geo, 2.5, 128);
  }

  data.m_bodies[0].set_geometry(&data.m_diku_geo);
  data.m_bodies[0].set_fixed(true);
  data.m_bodies[0].set_position(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
  quaternion_type Q;
  Q.Rz(-(165./180.)*value_traits::pi());
  data.m_bodies[0].set_orientation(Q);
  data.m_bodies[0].set_material_idx(20);
  data.m_configuration.add(&data.m_bodies[0]);

  vector3_type r;
  matrix3x3_type I;
  real_type density = 7200;
  real_type mass;
  vector3_type inertia_vec;

  OpenTissue::math::Random<real_type> my_random(value_traits::zero(),value_traits::one());
  int i=1;
  for(int z=0;z<5;++z)
    for(int y=0;y<40;++y)
      for(int x=0;x<5;++x)
      {
        r(0) = 15  - .025*i  - my_random()*5;
        r(1) = 15 + i*.2 + my_random()*.4 - .2;
        r(2) = -2.75 + my_random()*5.5;;

        data.m_bodies[i].set_position(r);
        data.m_bodies[i].set_geometry(&data.m_sphere[i%3]);
        data.m_bodies[i].attach(&data.m_gravity);
        data.m_configuration.add(&data.m_bodies[i]);
        data.m_bodies[i].set_material_idx(10);

        OpenTissue::geometry::compute_sphere_mass_properties(data.m_sphere[i%3].radius(),density,mass,inertia_vec);
        I= OpenTissue::math::diag(inertia_vec(0),inertia_vec(1),inertia_vec(2));

        data.m_bodies[i].set_mass(mass);
        data.m_bodies[i].set_inertia_bf(I);
        ++i;
      }

      data.m_material1.set_material_indices(10,10);
      data.m_material1.set_friction_coefficient(.01);
      data.m_material1.normal_restitution() = (.15);
      data.m_library.add(data.m_material1);

      data.m_material2.set_material_indices(10,20);
      data.m_material2.set_friction_coefficient(.04);
      data.m_material2.normal_restitution() = (.1);
      data.m_library.add(data.m_material2);

      data.m_gravity.set_acceleration(vector3_type(0,-9.81,0));
      data.m_simulator.init(data.m_configuration);
      material_type * default_material = data.m_library.default_material();
      default_material->set_friction_coefficient(.2);
      default_material->normal_restitution() = (.2);
      data.m_configuration.set_material_library(data.m_library);

      data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);

}

