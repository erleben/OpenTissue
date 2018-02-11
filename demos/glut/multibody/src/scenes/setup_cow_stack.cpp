//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_cow_stack.h"
#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/utility/utility_get_environment_variable.h>

void setup_cow_stack(Data & data)
{
  real_type const edge_resolution = 0.01;
  bool      const face_sampling = true;

  //--- Clear everything from configuration
  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();


  data.m_bodies.resize(10);


  //--- Prepare collision geometries 
  if(data.m_cow_mesh.size_vertices()==0)
  {
    std::string data_path = OpenTissue::utility::get_environment_variable("OPENTISSUE");

    OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/cow.obj",  data.m_cow_mesh);
    OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/support_box.obj",  data.m_support_box_mesh);
    OpenTissue::mesh::scale(data.m_support_box_mesh, 10.0, 10.0, 10.0);

    OpenTissue::mesh::compute_angle_weighted_vertex_normals(data.m_cow_mesh);
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(data.m_support_box_mesh);

    OpenTissue::sdf::semiauto_init_geometry(data.m_cow_mesh, edge_resolution, face_sampling, data.m_cow_geo,128);
    OpenTissue::sdf::semiauto_init_geometry(data.m_support_box_mesh, edge_resolution, face_sampling, data.m_support_box_geo,128);
  }

  //--- Setup objects in the configuration, attach geometries, forces etc..
  quaternion_type Q;
  vector3_type r;

  data.m_bodies[0].set_fixed(true);
  data.m_bodies[0].set_geometry(&data.m_support_box_geo);
  data.m_configuration.add(&data.m_bodies[0]);
  OpenTissue::math::Random<real_type> radian(value_traits::zero(),value_traits::two()*value_traits::pi());

  real_type e_n = 0.1;
  for(int i=1;i<10;++i)
  {
    r(2) = 1.25*(i%10);
    Q.Ru( radian(), r );
    data.m_bodies[i].attach(&data.m_gravity);
    data.m_bodies[i].set_position(r);
    data.m_bodies[i].set_orientation(Q);
    data.m_bodies[i].set_geometry(&data.m_cow_geo);
    data.m_configuration.add(&data.m_bodies[i]);
  }

  data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();

  default_material->set_friction_coefficient(.4);
  default_material->normal_restitution() = e_n;

  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
  data.m_simulator.get_stepper()->warm_starting()      = false;
  data.m_simulator.get_stepper()->use_stabilization()  = true;
  data.m_simulator.get_stepper()->use_friction()        = true;
  data.m_simulator.get_stepper()->use_bounce()         = true;
}

