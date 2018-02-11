//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include "setup_cow.h"
#include <OpenTissue/utility/utility_get_environment_variable.h>

void setup_cow(Data & data)
{
  data.m_configuration.clear();
  data.m_library.clear();
  data.m_simulator.clear();

  data.m_bodies.resize(2);

  //--- Prepare collision geometries 
  if(data.m_cow_mesh.size_vertices()==0)
  {
    std::string data_path = OpenTissue::utility::get_environment_variable("OPENTISSUE");

    OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/cow.obj",  data.m_cow_mesh);
    OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/support_box.obj",  data.m_support_box_mesh);
    OpenTissue::mesh::scale(data.m_support_box_mesh, 10.0, 10.0, 10.0);

    OpenTissue::mesh::compute_angle_weighted_vertex_normals(data.m_cow_mesh);
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(data.m_support_box_mesh);

    OpenTissue::sdf::semiauto_init_geometry(data.m_cow_mesh, 0.01, true, data.m_cow_geo,128);
    OpenTissue::sdf::semiauto_init_geometry(data.m_support_box_mesh, 0.01, true, data.m_support_box_geo,128);
  }

  quaternion_type Q;
  vector3_type r;

  data.m_bodies[0].set_fixed(true);
  data.m_bodies[0].set_geometry(&data.m_support_box_geo);
  data.m_configuration.add(&data.m_bodies[0]);

  data.m_bodies[1].attach(&data.m_gravity);
  r(2) = 1.5;
  data.m_bodies[1].set_position(r);
  Q.identity();
  data.m_bodies[1].set_orientation(Q);
  data.m_bodies[1].set_geometry(&data.m_cow_geo);
  data.m_configuration.add(&data.m_bodies[1]);

  data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));
  data.m_simulator.init(data.m_configuration);

  material_type * default_material = data.m_library.default_material();

  default_material->set_friction_coefficient(.4);
  default_material->normal_restitution() = 0.1;
  data.m_configuration.set_material_library(data.m_library);

  data.m_simulator.get_stepper()->get_solver()->set_max_iterations(10);
  data.m_simulator.get_stepper()->warm_starting()      = false;
  data.m_simulator.get_stepper()->use_stabilization()  = true;
  data.m_simulator.get_stepper()->use_friction()        = true;
  data.m_simulator.get_stepper()->use_bounce()         = true;
}
