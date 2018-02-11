//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/utility/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN

#include <OpenTissue/dynamics/mbd/math/mbd_default_math_policy.h>
#include <OpenTissue/dynamics/mbd/math/mbd_optimized_ublas_math_policy.h>
#include <OpenTissue/dynamics/mbd/mbd.h>
#include <OpenTissue/utility/utility_timer.h>
#include <OpenTissue/core/geometry/geometry_compute_box_mass_properties.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>
#include <OpenTissue/core/math/math_random.h>


class Application : public OpenTissue::glut::PerspectiveViewApplication
{
protected:

  template<typename types>
  class MyCollisionDetection
    : public OpenTissue::mbd::CollisionDetection<
    types
    , OpenTissue::mbd::SpatialHashing
    //, OpenTissue::mbd::SweepNPrune
    , OpenTissue::mbd::GeometryDispatcher
    , OpenTissue::mbd::SingleGroupAnalysis
    >
  {};

  template< typename types  >
  class MyStepper
    : public OpenTissue::mbd::ConstraintBasedShockPropagationStepper<
    types
    , OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy>
    >
  {};

  typedef OpenTissue::mbd::Types<
    //      OpenTissue::mbd::default_ublas_math_policy<double>
    OpenTissue::mbd::optimized_ublas_math_policy<double>
    , OpenTissue::mbd::NoSleepyPolicy
    , MyStepper
    , MyCollisionDetection
    , OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator
  > mbd_types;

protected:

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.

  typedef mbd_types::simulator_type                   simulator_type;
  typedef mbd_types::body_type                        body_type;
  typedef mbd_types::configuration_type               configuration_type;
  typedef mbd_types::material_library_type            material_library_type;
  typedef mbd_types::material_type                    material_type;

  typedef mbd_types::math_policy                       math_policy;
  typedef mbd_types::math_policy::index_type           size_type;
  typedef mbd_types::math_policy::real_type            real_type;
  typedef mbd_types::math_policy::vector3_type         vector3_type;
  typedef mbd_types::math_policy::quaternion_type      quaternion_type;
  typedef mbd_types::math_policy::matrix3x3_type       matrix3x3_type;
  typedef mbd_types::math_policy::vector_type          vector_type;
  typedef mbd_types::math_policy::matrix_type          matrix_type;
  typedef mbd_types::math_policy::value_traits         value_traits;

  typedef OpenTissue::mbd::Gravity<mbd_types>         gravity_type;
  typedef OpenTissue::mbd::Damping<mbd_types>         damping_type;

  typedef mbd_types::node_traits                      node_traits;


  typedef OpenTissue::geometry::Sphere<math_policy>     sphere_type;
  typedef OpenTissue::geometry::Plane<math_policy>      plane_type;
  typedef OpenTissue::geometry::OBB<math_policy>        box_type;
  typedef OpenTissue::polymesh::PolyMesh<math_policy>   mesh_type;
  typedef OpenTissue::grid::Grid<float,math_policy>            grid_type;
  typedef OpenTissue::sdf::Geometry<mesh_type,grid_type>  sdf_geometry_type;

  gravity_type                m_gravity;
  body_type                   m_bodies[300];
  simulator_type              m_simulator;
  configuration_type          m_configuration;
  material_library_type       m_library;
  box_type                    m_box;
  box_type                    m_big_box;
  box_type                    m_small_box;
  sphere_type                 m_sphere[3];
  mesh_type                   m_cow_mesh;
  mesh_type                   m_support_box_mesh;
  sdf_geometry_type           m_support_box_geo;
  sdf_geometry_type           m_cow_geo;

  std::string   m_prefix;         ///< Prefix std::string used for generating images...
  size_type     m_framecount;     ///< frame counter...
  bool          m_recording_on;  
  bool          m_wireframe_on;  
  bool          m_simulation_on; 
  bool          m_statistics_on; 
  size_type     m_max_frames;
  size_type     m_inbetween;      ///< frames inbetween a generated key frame
  real_type     m_running_time;
  real_type     m_timestep;

  vector_type  m_timings;
  vector_type  m_contacts;
  matrix_type  m_energy_kin;
  matrix_type  m_energy_pot;
  size_type m_cur;

protected:

  void reset_timestep()
  {
    m_timestep = 0.01;
    m_cur = 0;
    m_framecount = 0;
    m_max_frames = boost::numeric_cast<size_type>( std::floor( m_running_time / m_timestep ) );
    m_inbetween = boost::numeric_cast<size_type>( std::floor( 0.1 / m_timestep ) );
  }

  void increment_timestep()
  {
    m_timestep *= 10.0;
    m_cur = 0;
    m_framecount = 0;
    m_max_frames = boost::numeric_cast<size_type>( std::floor( m_running_time / m_timestep ) );
    m_inbetween =  boost::numeric_cast<size_type>( std::floor( 0.1 / m_timestep ) );
  }

  void decrement_timestep()
  {
    m_timestep /= 10.0;
    m_cur = 0;
    m_framecount = 0;
    m_max_frames = boost::numeric_cast<size_type>( std::floor( m_running_time / m_timestep ) );
    m_inbetween =  boost::numeric_cast<size_type>( std::floor( 0.1 / m_timestep ) );
  }

  void setup_bowling()
  {
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    if(m_cow_mesh.size_vertices()==0)
    {
      std::string data_path = opentissue_path;

      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/cow.obj",  m_cow_mesh);
      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/support_box.obj",  m_support_box_mesh);
      OpenTissue::mesh::scale(m_support_box_mesh, 10.0, 10.0, 10.0);

      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_cow_mesh);
      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_support_box_mesh);

      OpenTissue::sdf::semiauto_init_geometry(m_cow_mesh, 0.01, true, m_cow_geo,128);
      OpenTissue::sdf::semiauto_init_geometry(m_support_box_mesh, 0.01, true, m_support_box_geo,128);
    }

    OpenTissue::math::Random<real_type> radian(value_traits::zero(),value_traits::two()*value_traits::pi());

    quaternion_type Q;
    vector3_type r;

    m_bodies[0].set_fixed(true);
    m_bodies[0].set_geometry(&m_support_box_geo);
    m_configuration.add(&m_bodies[0]);
    for(int i=1;i<50;++i)
    {
      int j = i-1;
      r(0) = 1.25*(j%7) -4.0;
      r(1) = 1.25*std::floor(j/7.0) - 4.0;
      r(2) = 1.25;
      Q.Rz( radian() );
      m_bodies[i].attach(&m_gravity);
      m_bodies[i].set_position(r);
      m_bodies[i].set_orientation(Q);
      m_bodies[i].set_geometry(&m_cow_geo);
      m_configuration.add(&m_bodies[i]);
    }
    m_sphere[0].radius(0.25);
    m_bodies[50].attach(&m_gravity);
    m_bodies[50].set_position(vector3_type(6.0,value_traits::zero(),1.75));
    m_bodies[50].set_velocity(vector3_type(-12,0,0));
    m_bodies[50].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[50].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[50]);

    m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(0.4);
    default_material->normal_restitution() = 0.5;
    m_configuration.set_material_library(m_library);
    m_timestep = 0.01;
    m_max_frames = 3000;
    m_inbetween = 100;
  }

  void setup_cow()
  {
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    //--- Prepare collision geometries 
    if(m_cow_mesh.size_vertices()==0)
    {
      std::string data_path = opentissue_path;

      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/cow.obj",  m_cow_mesh);
      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/support_box.obj",  m_support_box_mesh);
      OpenTissue::mesh::scale(m_support_box_mesh, 10.0, 10.0, 10.0);

      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_cow_mesh);
      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_support_box_mesh);

      OpenTissue::sdf::semiauto_init_geometry(m_cow_mesh, 0.01, true, m_cow_geo,128);
      OpenTissue::sdf::semiauto_init_geometry(m_support_box_mesh, 0.01, true, m_support_box_geo,128);
    }

    quaternion_type Q;
    vector3_type r;

    m_bodies[0].set_fixed(true);
    m_bodies[0].set_geometry(&m_support_box_geo);
    m_configuration.add(&m_bodies[0]);

    m_bodies[1].attach(&m_gravity);
    r(2) = 1.5;
    m_bodies[1].set_position(r);
    Q.identity();
    m_bodies[1].set_orientation(Q);
    m_bodies[1].set_geometry(&m_cow_geo);
    m_configuration.add(&m_bodies[1]);

    m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(.4);
    default_material->normal_restitution() = 0.1;
    m_configuration.set_material_library(m_library);
    m_timestep = 0.01;
    m_max_frames = 3000;
    m_inbetween = 100;
  }

  void setup_box()
  {
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    quaternion_type Q;
    matrix3x3_type R;
    R = OpenTissue::math::diag(value_traits::one());

    m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(10,10,1));
    m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(1,1,1));

    m_bodies[0].set_position(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_geometry(&m_box);
    m_bodies[0].set_fixed(true);
    m_configuration.add(&m_bodies[0]);

    m_bodies[1].attach(&m_gravity);
    m_bodies[1].set_position(vector3_type(value_traits::zero(),0,2.5));
    m_bodies[1].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_geometry(&m_small_box);
    m_configuration.add(&m_bodies[1]);

    m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(0.4);
    default_material->normal_restitution() = 0.1;
    m_configuration.set_material_library(m_library);
    m_timestep = 0.01;
  }

  void setup_cow_stack(real_type const & edge_resolution,bool const & face_sampling)
  {
    //--- Clear everything from configuration
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    //--- Prepare collision geometries 
    if(m_cow_mesh.size_vertices()==0)
    {
      std::string data_path = opentissue_path;

      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/cow.obj",  m_cow_mesh);
      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/support_box.obj",  m_support_box_mesh);
      OpenTissue::mesh::scale(m_support_box_mesh, 10.0, 10.0, 10.0);

      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_cow_mesh);
      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_support_box_mesh);

      OpenTissue::sdf::semiauto_init_geometry(m_cow_mesh, edge_resolution, face_sampling, m_cow_geo,128);
      OpenTissue::sdf::semiauto_init_geometry(m_support_box_mesh, edge_resolution, face_sampling, m_support_box_geo,128);
    }

    //--- Setup objects in the configuration, attach geometries, forces etc..
    quaternion_type Q;
    vector3_type r;

    m_bodies[0].set_fixed(true);
    m_bodies[0].set_geometry(&m_support_box_geo);
    m_configuration.add(&m_bodies[0]);
    OpenTissue::math::Random<real_type> radian(value_traits::zero(),value_traits::two()*value_traits::pi());

    real_type e_n = 0.1;
    for(int i=1;i<10;++i)
    {
      r(2) = 1.25*(i%10);
      Q.Ru( radian(), r );
      m_bodies[i].attach(&m_gravity);
      m_bodies[i].set_position(r);
      m_bodies[i].set_orientation(Q);
      m_bodies[i].set_geometry(&m_cow_geo);
      m_configuration.add(&m_bodies[i]);
    }

    m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(.4);
    default_material->normal_restitution() = e_n;
    m_configuration.set_material_library(m_library);
  }

  void setup_falling_cows()
  {
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    if(m_cow_mesh.size_vertices()==0)
    {
      real_type const & edge_resolution = 0.01;
      bool const & face_sampling = true;

      std::string data_path = opentissue_path;

      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/cow.obj",  m_cow_mesh);
      OpenTissue::mesh::obj_read(data_path + "/demos/data/obj/support_box.obj",  m_support_box_mesh);
      OpenTissue::mesh::scale(m_support_box_mesh, 10.0, 10.0, 10.0);

      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_cow_mesh);
      OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_support_box_mesh);

      OpenTissue::sdf::semiauto_init_geometry(m_cow_mesh, edge_resolution, face_sampling, m_cow_geo,128);
      OpenTissue::sdf::semiauto_init_geometry(m_support_box_mesh, edge_resolution, face_sampling, m_support_box_geo,128);
    }

    m_bodies[0].set_fixed(true);
    m_bodies[0].set_geometry(&m_support_box_geo);
    m_configuration.add(&m_bodies[0]);

    for(int i=1;i<250;++i)
    {
      m_bodies[i].attach(&m_gravity);
      m_bodies[i].set_geometry(&m_cow_geo);
      m_configuration.add(&m_bodies[i]);
    }

    m_bodies[0].set_position( vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()) );
    m_bodies[0].set_orientation( quaternion_type(1,0,0,0) );
    m_bodies[1].set_position( vector3_type(1.18839,-3.76415,3.75) );
    m_bodies[1].set_orientation( quaternion_type(0.716584,0.152243,-0.482221,0.480408) );
    m_bodies[2].set_position( vector3_type(0.396619,-0.184698,5) );
    m_bodies[2].set_orientation( quaternion_type(0.665121,0.0590083,-0.0274791,0.743893) );
    m_bodies[3].set_position( vector3_type(0.455702,-3.28147,6.25) );
    m_bodies[3].set_orientation( quaternion_type(0.0846058,0.0641904,-0.46223,0.880378) );
    m_bodies[4].set_position( vector3_type(-3.78686,-2.42012,7.5) );
    m_bodies[4].set_orientation( quaternion_type(0.0993081,-0.430969,-0.275425,0.853548) );
    m_bodies[5].set_position( vector3_type(3.31468,-3.29832,8.75) );
    m_bodies[5].set_orientation( quaternion_type(0.916559,0.133608,-0.132948,0.352694) );
    m_bodies[6].set_position( vector3_type(3.28538,-2.19306,10) );
    m_bodies[6].set_orientation( quaternion_type(-0.853452,0.159251,-0.106303,0.484725) );
    m_bodies[7].set_position( vector3_type(3.25657,-0.133671,11.25) );
    m_bodies[7].set_orientation( quaternion_type(-0.617701,0.218654,-0.00897499,0.755351) );
    m_bodies[8].set_position( vector3_type(-2.4228,-2.74239,2.5) );
    m_bodies[8].set_orientation( quaternion_type(0.856188,-0.282454,-0.319712,0.291454) );
    m_bodies[9].set_position( vector3_type(-1.42912,-3.50047,3.75) );
    m_bodies[9].set_orientation( quaternion_type(-0.941411,-0.0905098,-0.221694,0.237497) );
    m_bodies[10].set_position( vector3_type(3.68017,-0.184454,5) );
    m_bodies[10].set_orientation( quaternion_type(-0.32609,0.560128,-0.0280742,0.761009) );
    m_bodies[11].set_position( vector3_type(2.25239,-2.22407,6.25) );
    m_bodies[11].set_orientation( quaternion_type(0.960401,0.0895764,-0.08845,0.248559) );
    m_bodies[12].set_position( vector3_type(1.2172,1.26701,7.5) );
    m_bodies[12].set_orientation( quaternion_type(0.42388,0.143118,0.148974,0.881845) );
    m_bodies[13].set_position( vector3_type(-3.09568,3.03952,8.75) );
    m_bodies[13].set_orientation( quaternion_type(-0.518346,-0.271063,0.266146,0.766165) );
    m_bodies[14].set_position( vector3_type(0.250374,3.08347,10) );
    m_bodies[14].set_orientation( quaternion_type(0.537561,0.0201691,0.248391,0.805558) );
    m_bodies[15].set_position( vector3_type(2.64791,-3.91626,11.25) );
    m_bodies[15].set_orientation( quaternion_type(-0.731071,0.148053,-0.21897,0.629022) );
    m_bodies[16].set_position( vector3_type(-2.3144,-0.798242,2.5) );
    m_bodies[16].set_orientation( quaternion_type(0.908122,-0.276945,-0.0955188,0.299154) );
    m_bodies[17].set_position( vector3_type(2.0268,2.43501,3.75) );
    m_bodies[17].set_orientation( quaternion_type(-0.509298,0.355304,0.426866,0.657388) );
    m_bodies[18].set_position( vector3_type(-1.3195,-0.193731,5) );
    m_bodies[18].set_orientation( quaternion_type(-0.999994,-0.000904542,-0.000132807,0.0034276) );
    m_bodies[19].set_position( vector3_type(1.91327,-0.369274,6.25) );
    m_bodies[19].set_orientation( quaternion_type(-0.998538,0.0157955,-0.00304864,0.0515985) );
    m_bodies[20].set_position( vector3_type(-0.950346,-3.89087,7.5) );
    m_bodies[20].set_orientation( quaternion_type(-0.999818,-0.00213244,-0.00873054,0.0168289) );
    m_bodies[21].set_position( vector3_type(0.415662,0.503067,8.75) );
    m_bodies[21].set_orientation( quaternion_type(0.736153,0.0320626,0.0388046,0.674941) );
    m_bodies[22].set_position( vector3_type(0.671285,2.13276,10) );
    m_bodies[22].set_orientation( quaternion_type(0.925081,0.0248791,0.0790438,0.370618) );
    m_bodies[23].set_position( vector3_type(2.7839,-2.54073,11.25) );
    m_bodies[23].set_orientation( quaternion_type(0.725882,0.16139,-0.147293,0.652192) );
    m_bodies[24].set_position( vector3_type(3.89306,-2.51045,2.5) );
    m_bodies[24].set_orientation( quaternion_type(-0.975994,0.161079,-0.103873,0.10344) );
    m_bodies[25].set_position( vector3_type(-1.68499,-0.236946,3.75) );
    m_bodies[25].set_orientation( quaternion_type(0.997956,-0.026149,-0.00367711,0.0581955) );
    m_bodies[26].set_position( vector3_type(0.938627,3.16477,5) );
    m_bodies[26].set_orientation( quaternion_type(-0.700004,0.111879,0.377222,0.595971) );
    m_bodies[27].set_position( vector3_type(2.26093,3.81616,6.25) );
    m_bodies[27].set_orientation( quaternion_type(0.378217,0.273092,0.460943,0.75492) );
    m_bodies[28].set_position( vector3_type(2.31416,2.29609,7.5) );
    m_bodies[28].set_orientation( quaternion_type(-0.252323,0.273822,0.271684,0.887436) );
    m_bodies[29].set_position( vector3_type(2.73336,0.438368,8.75) );
    m_bodies[29].set_orientation( quaternion_type(0.964991,0.0781166,0.0125281,0.250066) );
    m_bodies[30].set_position( vector3_type(0.21424,1.95697,10) );
    m_bodies[30].set_orientation( quaternion_type(-0.35825,0.0196253,0.179267,0.916043) );
    m_bodies[31].set_position( vector3_type(-3.3889,3.15061,11.25) );
    m_bodies[31].set_orientation( quaternion_type(-0.603656,-0.222105,0.206488,0.737314) );
    m_bodies[32].set_position( vector3_type(-3.07224,3.93554,2.5) );
    m_bodies[32].set_orientation( quaternion_type(0.476518,-0.483734,0.619665,0.393634) );
    m_bodies[33].set_position( vector3_type(2.44575,1.45476,3.75) );
    m_bodies[33].set_orientation( quaternion_type(0.994534,0.0542461,0.0322661,0.0831739) );
    m_bodies[34].set_position( vector3_type(2.24531,-3.48265,5) );
    m_bodies[34].set_orientation( quaternion_type(0.575079,0.282864,-0.438745,0.629901) );
    m_bodies[35].set_position( vector3_type(2.36738,-0.884671,6.25) );
    m_bodies[35].set_orientation( quaternion_type(0.887721,0.161667,-0.0604135,0.426808) );
    m_bodies[36].set_position( vector3_type(-0.135868,-0.733055,7.5) );
    m_bodies[36].set_orientation( quaternion_type(0.0255484,-0.0180211,-0.0972295,0.994771) );
    m_bodies[37].set_position( vector3_type(-3.00803,1.44108,8.75) );
    m_bodies[37].set_orientation( quaternion_type(0.908683,-0.134108,0.0642486,0.390106) );
    m_bodies[38].set_position( vector3_type(2.79513,0.80752,10) );
    m_bodies[38].set_orientation( quaternion_type(0.507234,0.231296,0.066822,0.827497) );
    m_bodies[39].set_position( vector3_type(-2.67281,-0.433973,11.25) );
    m_bodies[39].set_orientation( quaternion_type(0.412122,-0.210458,-0.0341712,0.88583) );
    m_bodies[40].set_position( vector3_type(-2.23139,-2.39644,2.5) );
    m_bodies[40].set_orientation( quaternion_type(0.534729,-0.457697,-0.49155,0.512793) );
    m_bodies[41].set_position( vector3_type(1.67864,-0.269906,3.75) );
    m_bodies[41].set_orientation( quaternion_type(0.134062,0.404012,-0.0649603,0.902542) );
    m_bodies[42].set_position( vector3_type(2.6418,-2.25849,5) );
    m_bodies[42].set_orientation( quaternion_type(0.625586,0.338464,-0.289354,0.640592) );
    m_bodies[43].set_position( vector3_type(-2.44502,1.56169,6.25) );
    m_bodies[43].set_orientation( quaternion_type(-0.887279,-0.163663,0.104535,0.418357) );
    m_bodies[44].set_position( vector3_type(-3.197,-2.28877,7.5) );
    m_bodies[44].set_orientation( quaternion_type(0.703487,-0.268315,-0.19209,0.629455) );
    m_bodies[45].set_position( vector3_type(-3.42357,-2.64302,8.75) );
    m_bodies[45].set_orientation( quaternion_type(-0.995837,-0.0319706,-0.0246816,0.0817108) );
    m_bodies[46].set_position( vector3_type(-3.42015,-3.81323,10) );
    m_bodies[46].set_orientation( quaternion_type(-0.990418,-0.0420379,-0.0468693,0.122912) );
    m_bodies[47].set_position( vector3_type(-1.62029,0.363903,11.25) );
    m_bodies[47].set_orientation( quaternion_type(0.966705,-0.0364601,0.00818861,0.25315) );
    m_bodies[48].set_position( vector3_type(3.64428,-3.19138,2.5) );
    m_bodies[48].set_orientation( quaternion_type(-0.99538,0.0641894,-0.0562122,0.0440344) );
    m_bodies[49].set_position( vector3_type(3.52342,-1.93963,3.75) );
    m_bodies[49].set_orientation( quaternion_type(-0.579228,0.522306,-0.287528,0.555894) );
    m_bodies[50].set_position( vector3_type(2.41914,-0.0357677,5) );
    m_bodies[50].set_orientation( quaternion_type(0.142988,0.431046,-0.00637313,0.890906) );
    m_bodies[51].set_position( vector3_type(3.39622,-1.56194,6.25) );
    m_bodies[51].set_orientation( quaternion_type(-0.469168,0.411835,-0.189405,0.757892) );
    m_bodies[52].set_position( vector3_type(-1.51531,-0.431532,7.5) );
    m_bodies[52].set_orientation( quaternion_type(-0.132542,-0.19598,-0.0558117,0.970005) );
    m_bodies[53].set_position( vector3_type(3.1218,1.2819,8.75) );
    m_bodies[53].set_orientation( quaternion_type(-0.842063,0.179547,0.073727,0.503246) );
    m_bodies[54].set_position( vector3_type(1.9848,0.846828,10) );
    m_bodies[54].set_orientation( quaternion_type(0.486936,0.169459,0.0723009,0.853785) );
    m_bodies[55].set_position( vector3_type(-2.53487,-1.54778,11.25) );
    m_bodies[55].set_orientation( quaternion_type(0.365757,-0.202762,-0.123805,0.899878) );
    m_bodies[56].set_position( vector3_type(-3.96728,3.11545,2.5) );
    m_bodies[56].set_orientation( quaternion_type(-0.152754,-0.696415,0.546885,0.438849) );
    m_bodies[57].set_position( vector3_type(3.73998,3.16623,3.75) );
    m_bodies[57].set_orientation( quaternion_type(-0.831246,0.336938,0.285248,0.33784) );
    m_bodies[58].set_position( vector3_type(2.11811,3.562,5) );
    m_bodies[58].set_orientation( quaternion_type(-0.678684,0.239538,0.402829,0.565454) );
    m_bodies[59].set_position( vector3_type(-0.286996,-3.72192,6.25) );
    m_bodies[59].set_orientation( quaternion_type(0.229619,-0.0383695,-0.497596,0.835584) );
    m_bodies[60].set_position( vector3_type(2.51241,-1.97211,7.5) );
    m_bodies[60].set_orientation( quaternion_type(0.80519,0.18277,-0.143465,0.545602) );
    m_bodies[61].set_position( vector3_type(-1.48796,-2.43867,8.75) );
    m_bodies[61].set_orientation( quaternion_type(-0.215598,-0.157853,-0.258711,0.928261) );
    m_bodies[62].set_position( vector3_type(-0.324351,2.78951,10) );
    m_bodies[62].set_orientation( quaternion_type(0.0638584,-0.0311633,0.268014,0.960791) );
    m_bodies[63].set_position( vector3_type(-3.46873,-2.01044,11.25) );
    m_bodies[63].set_orientation( quaternion_type(0.193356,-0.284959,-0.165159,0.924194) );
    m_bodies[64].set_position( vector3_type(-3.33714,1.33781,2.5) );
    m_bodies[64].set_orientation( quaternion_type(0.0784902,-0.759715,0.304559,0.569137) );
    m_bodies[65].set_position( vector3_type(2.33808,1.01822,3.75) );
    m_bodies[65].set_orientation( quaternion_type(-0.0673981,0.514396,0.224016,0.825028) );
    m_bodies[66].set_position( vector3_type(2.52998,0.162236,5) );
    m_bodies[66].set_orientation( quaternion_type(0.781383,0.281635,0.0180599,0.556594) );
    m_bodies[67].set_position( vector3_type(2.57808,-3.13547,6.25) );
    m_bodies[67].set_orientation( quaternion_type(-0.498117,0.299963,-0.364816,0.727194) );
    m_bodies[68].set_position( vector3_type(-1.26041,1.52947,7.5) );
    m_bodies[68].set_orientation( quaternion_type(0.759819,-0.105633,0.128181,0.628559) );
    m_bodies[69].set_position( vector3_type(0.882717,-2.20917,8.75) );
    m_bodies[69].set_orientation( quaternion_type(-0.745235,0.0649119,-0.162455,0.643444) );
    m_bodies[70].set_position( vector3_type(2.5688,-2.7336,10) );
    m_bodies[70].set_orientation( quaternion_type(-0.121986,0.238719,-0.254034,0.9293) );
    m_bodies[71].set_position( vector3_type(1.77557,-1.61614,11.25) );
    m_bodies[71].set_orientation( quaternion_type(0.961752,0.0422802,-0.0384839,0.267888) );
    m_bodies[72].set_position( vector3_type(3.51341,-0.459365,2.5) );
    m_bodies[72].set_orientation( quaternion_type(0.534729,0.684637,-0.0895136,0.48716) );
    m_bodies[73].set_position( vector3_type(3.57738,-3.16306,3.75) );
    m_bodies[73].set_orientation( quaternion_type(0.814192,0.342082,-0.302464,0.358589) );
    m_bodies[74].set_position( vector3_type(-3.30467,-3.35862,5) );
    m_bodies[74].set_orientation( quaternion_type(0.927322,-0.180023,-0.182963,0.272378) );
    m_bodies[75].set_position( vector3_type(0.31312,-2.76876,6.25) );
    m_bodies[75].set_orientation( quaternion_type(-0.115798,0.0454499,-0.401891,0.907198) );
    m_bodies[76].set_position( vector3_type(1.27018,1.76214,7.5) );
    m_bodies[76].set_orientation( quaternion_type(-0.734528,0.110386,0.153139,0.651791) );
    m_bodies[77].set_position( vector3_type(0.618305,0.816553,8.75) );
    m_bodies[77].set_orientation( quaternion_type(-0.354936,0.0656146,0.0866527,0.928551) );
    m_bodies[78].set_position( vector3_type(-0.170293,2.77999,10) );
    m_bodies[78].set_orientation( quaternion_type(-0.861687,-0.00832452,0.135896,0.488834) );
    m_bodies[79].set_position( vector3_type(-0.355113,2.29365,11.25) );
    m_bodies[79].set_orientation( quaternion_type(-0.847959,-0.0163866,0.10584,0.519129) );
    m_bodies[80].set_position( vector3_type(-1.1879,-2.96213,2.5) );
    m_bodies[80].set_orientation( quaternion_type(-0.833796,-0.161766,-0.403377,0.340445) );
    m_bodies[81].set_position( vector3_type(-0.0323496,-0.906644,3.75) );
    m_bodies[81].set_orientation( quaternion_type(-0.916636,-0.00335156,-0.0939321,0.388516) );
    m_bodies[82].set_position( vector3_type(1.90594,-0.872951,5) );
    m_bodies[82].set_orientation( quaternion_type(-0.895363,0.156554,-0.0717042,0.4107) );
    m_bodies[83].set_position( vector3_type(1.49089,-2.3393,6.25) );
    m_bodies[83].set_orientation( quaternion_type(-0.901841,0.0942047,-0.147813,0.394918) );
    m_bodies[84].set_position( vector3_type(0.457167,-2.33442,7.5) );
    m_bodies[84].set_orientation( quaternion_type(0.928751,0.0215391,-0.109985,0.353357) );
    m_bodies[85].set_position( vector3_type(0.353404,0.734031,8.75) );
    m_bodies[85].set_orientation( quaternion_type(-0.995687,0.0037308,0.00774897,0.0923715) );
    m_bodies[86].set_position( vector3_type(-3.12546,-2.91842,10) );
    m_bodies[86].set_orientation( quaternion_type(-0.853652,-0.149677,-0.139762,0.478897) );
    m_bodies[87].set_position( vector3_type(-3.16355,-0.373669,11.25) );
    m_bodies[87].set_orientation( quaternion_type(-0.782817,-0.168361,-0.0198862,0.598713) );
    m_bodies[88].set_position( vector3_type(-2.96188,0.394665,2.5) );
    m_bodies[88].set_orientation( quaternion_type(-0.876755,-0.36563,0.0487195,0.308612) );
    m_bodies[89].set_position( vector3_type(-3.73144,3.50829,3.75) );
    m_bodies[89].set_orientation( quaternion_type(0.764598,-0.378864,0.356207,0.380749) );
    m_bodies[90].set_position( vector3_type(3.70238,-3.00436,5) );
    m_bodies[90].set_orientation( quaternion_type(-0.990668,0.0730377,-0.0592677,0.098636) );
    m_bodies[91].set_position( vector3_type(2.93332,1.6867,6.25) );
    m_bodies[91].set_orientation( quaternion_type(0.947797,0.131608,0.0756766,0.280417) );
    m_bodies[92].set_position( vector3_type(2.90426,3.42918,7.5) );
    m_bodies[92].set_orientation( quaternion_type(-0.736023,0.224867,0.265509,0.580698) );
    m_bodies[93].set_position( vector3_type(-1.02725,2.88888,8.75) );
    m_bodies[93].set_orientation( quaternion_type(0.464419,-0.0981219,0.275942,0.835789) );
    m_bodies[94].set_position( vector3_type(-0.630024,-2.82614,10) );
    m_bodies[94].set_orientation( quaternion_type(0.889305,-0.0276751,-0.124144,0.439271) );
    m_bodies[95].set_position( vector3_type(-3.70238,-0.45912,11.25) );
    m_bodies[95].set_orientation( quaternion_type(-0.777479,-0.196454,-0.0243616,0.596942) );
    m_bodies[96].set_position( vector3_type(-3.08029,-0.574847,2.5) );
    m_bodies[96].set_orientation( quaternion_type(0.184318,-0.755261,-0.140947,0.612978) );
    m_bodies[97].set_position( vector3_type(1.4904,1.05167,3.75) );
    m_bodies[97].set_orientation( quaternion_type(-0.311551,0.339613,0.23964,0.854501) );
    m_bodies[98].set_position( vector3_type(3.87548,2.87448,5) );
    m_bodies[98].set_orientation( quaternion_type(0.908001,0.233676,0.173319,0.30148) );
    m_bodies[99].set_position( vector3_type(-2.2873,0.0919218,6.25) );
    m_bodies[99].set_orientation( quaternion_type(0.295106,-0.328339,0.0131953,0.89718) );
    m_bodies[100].set_position( vector3_type(-0.535051,0.492325,7.5) );
    m_bodies[100].set_orientation( quaternion_type(0.564204,-0.058626,0.0539445,0.821782) );
    m_bodies[101].set_position( vector3_type(-0.388318,-0.123417,8.75) );
    m_bodies[101].set_orientation( quaternion_type(-0.963106,-0.0119305,-0.0037918,0.268831) );
    m_bodies[102].set_position( vector3_type(-2.76876,3.30857,10) );
    m_bodies[102].set_orientation( quaternion_type(-0.281427,-0.243951,0.291513,0.881083) );
    m_bodies[103].set_position( vector3_type(-2.77413,-1.55461,11.25) );
    m_bodies[103].set_orientation( quaternion_type(0.653661,-0.179579,-0.100636,0.728252) );
    m_bodies[104].set_position( vector3_type(-2.56441,-0.806543,2.5) );
    m_bodies[104].set_orientation( quaternion_type(-0.833213,-0.386263,-0.121485,0.376561) );
    m_bodies[105].set_position( vector3_type(-3.10764,1.94525,3.75) );
    m_bodies[105].set_orientation( quaternion_type(0.999995,-0.00187482,0.00117356,0.00226235) );
    m_bodies[106].set_position( vector3_type(-3.53392,-3.35984,5) );
    m_bodies[106].set_orientation( quaternion_type(0.688897,-0.366777,-0.34871,0.518938) );
    m_bodies[107].set_position( vector3_type(-0.848537,2.91012,6.25) );
    m_bodies[107].set_orientation( quaternion_type(-0.153797,-0.120703,0.41396,0.889052) );
    m_bodies[108].set_position( vector3_type(-2.75704,1.54485,7.5) );
    m_bodies[108].set_orientation( quaternion_type(-0.107319,-0.336802,0.18872,0.916205) );
    m_bodies[109].set_position( vector3_type(1.65423,3.5432,8.75) );
    m_bodies[109].set_orientation( quaternion_type(0.00704688,0.172598,0.36969,0.912956) );
    m_bodies[110].set_position( vector3_type(-3.37547,-1.0126,10) );
    m_bodies[110].set_orientation( quaternion_type(0.965292,-0.0831459,-0.0249429,0.246324) );
    m_bodies[111].set_position( vector3_type(3.06858,3.9978,11.25) );
    m_bodies[111].set_orientation( quaternion_type(0.394576,0.228729,0.297993,0.838566) );
    m_bodies[112].set_position( vector3_type(-2.18598,-3.9856,2.5) );
    m_bodies[112].set_orientation( quaternion_type(0.558094,-0.349642,-0.637485,0.399868) );
    m_bodies[113].set_position( vector3_type(-0.0155034,3.65893,3.75) );
    m_bodies[113].set_orientation( quaternion_type(0.970099,-0.000718186,0.169498,0.173717) );
    m_bodies[114].set_position( vector3_type(-0.234016,-2.58003,5) );
    m_bodies[114].set_orientation( quaternion_type(0.493454,-0.0361446,-0.398496,0.772268) );
    m_bodies[115].set_position( vector3_type(0.821924,-0.576312,6.25) );
    m_bodies[115].set_orientation( quaternion_type(-0.998118,0.00796236,-0.00558299,0.0605466) );
    m_bodies[116].set_position( vector3_type(3.59178,2.52559,7.5) );
    m_bodies[116].set_orientation( quaternion_type(0.396337,0.379441,0.266807,0.79231) );
    m_bodies[117].set_position( vector3_type(0.73574,-3.01608,8.75) );
    m_bodies[117].set_orientation( quaternion_type(-0.25631,0.0765974,-0.314002,0.910956) );
    m_bodies[118].set_position( vector3_type(3.96631,-1.03775,10) );
    m_bodies[118].set_orientation( quaternion_type(-0.654386,0.2775,-0.0726056,0.699643) );
    m_bodies[119].set_position( vector3_type(-2.47774,3.10642,11.25) );
    m_bodies[119].set_orientation( quaternion_type(-0.888514,-0.0952892,0.119467,0.432654) );
    m_bodies[120].set_position( vector3_type(-2.46602,1.4528,2.5) );
    m_bodies[120].set_orientation( quaternion_type(0.171487,-0.639297,0.376629,0.648107) );
    m_bodies[121].set_position( vector3_type(0.526261,-1.08634,3.75) );
    m_bodies[121].set_orientation( quaternion_type(0.901177,0.0579031,-0.119527,0.412603) );
    m_bodies[122].set_position( vector3_type(0.315806,-0.40907,5) );
    m_bodies[122].set_orientation( quaternion_type(-0.139571,0.0622115,-0.0805839,0.984965) );
    m_bodies[123].set_position( vector3_type(0.546037,-1.83319,6.25) );
    m_bodies[123].set_orientation( quaternion_type(-0.805929,0.0494574,-0.166041,0.566094) );
    m_bodies[124].set_position( vector3_type(2.11469,-1.37565,7.5) );
    m_bodies[124].set_orientation( quaternion_type(-0.956267,0.0781677,-0.0508498,0.277231) );
    m_bodies[125].set_position( vector3_type(-3.2595,-1.45476,8.75) );
    m_bodies[125].set_orientation( quaternion_type(0.571544,-0.283031,-0.12632,0.759786) );
    m_bodies[126].set_position( vector3_type(1.92181,1.23551,10) );
    m_bodies[126].set_orientation( quaternion_type(-0.753992,0.12307,0.07912,0.640383) );
    m_bodies[127].set_position( vector3_type(0.535051,1.92938,11.25) );
    m_bodies[127].set_orientation( quaternion_type(0.913699,0.019029,0.0686182,0.400105) );
    m_bodies[128].set_position( vector3_type(-3.67675,2.32221,2.5) );
    m_bodies[128].set_orientation( quaternion_type(-0.930412,-0.268653,0.16968,0.18267) );
    m_bodies[129].set_position( vector3_type(0.527726,-0.319712,3.75) );
    m_bodies[129].set_orientation( quaternion_type(0.873181,0.0676798,-0.0410024,0.48093) );
    m_bodies[130].set_position( vector3_type(0.9064,-3.22068,5) );
    m_bodies[130].set_orientation( quaternion_type(0.780545,0.0941779,-0.334639,0.519516) );
    m_bodies[131].set_position( vector3_type(0.396374,2.92209,6.25) );
    m_bodies[131].set_orientation( quaternion_type(-0.726739,0.0393989,0.29045,0.621239) );
    m_bodies[132].set_position( vector3_type(0.665426,0.280648,7.5) );
    m_bodies[132].set_orientation( quaternion_type(0.355742,0.0825377,0.0348109,0.930281) );
    m_bodies[133].set_position( vector3_type(1.80364,-2.17743,8.75) );
    m_bodies[133].set_orientation( quaternion_type(0.111321,0.194925,-0.235322,0.94564) );
    m_bodies[134].set_position( vector3_type(-2.59249,3.05905,10) );
    m_bodies[134].set_orientation( quaternion_type(-0.388489,-0.221724,0.261628,0.855258) );
    m_bodies[135].set_position( vector3_type(-3.69652,1.87518,11.25) );
    m_bodies[135].set_orientation( quaternion_type(0.561907,-0.255042,0.129378,0.776193) );
    m_bodies[136].set_position( vector3_type(2.16108,0.687643,2.5) );
    m_bodies[136].set_orientation( quaternion_type(0.186579,0.629007,0.200147,0.727654) );
    m_bodies[137].set_position( vector3_type(-1.7514,-2.82076,3.75) );
    m_bodies[137].set_orientation( quaternion_type(-0.727594,-0.239879,-0.386345,0.513617) );
    m_bodies[138].set_position( vector3_type(-3.42112,-2.13715,5) );
    m_bodies[138].set_orientation( quaternion_type(-0.866225,-0.266081,-0.166218,0.388879) );
    m_bodies[139].set_position( vector3_type(-1.30485,-3.27732,6.25) );
    m_bodies[139].set_orientation( quaternion_type(0.650244,-0.13813,-0.346934,0.661619) );
    m_bodies[140].set_position( vector3_type(-0.65273,1.184,7.5) );
    m_bodies[140].set_orientation( quaternion_type(0.795651,-0.0518831,0.0941114,0.596147) );
    m_bodies[141].set_position( vector3_type(0.309702,3.66137,8.75) );
    m_bodies[141].set_orientation( quaternion_type(-0.0262193,0.0326226,0.385672,0.921686) );
    m_bodies[142].set_position( vector3_type(-2.41035,0.934233,10) );
    m_bodies[142].set_orientation( quaternion_type(-0.900886,-0.101293,0.0392604,0.420242) );
    m_bodies[143].set_position( vector3_type(-2.72457,-0.398328,11.25) );
    m_bodies[143].set_orientation( quaternion_type(0.930622,-0.0860934,-0.0125867,0.355487) );
    m_bodies[144].set_position( vector3_type(-0.724509,-2.13642,2.5) );
    m_bodies[144].set_orientation( quaternion_type(0.948041,-0.0684514,-0.201848,0.236199) );
    m_bodies[145].set_position( vector3_type(-3.80151,1.88177,3.75) );
    m_bodies[145].set_orientation( quaternion_type(0.782041,-0.41846,0.207141,0.412791) );
    m_bodies[146].set_position( vector3_type(-1.37541,2.96847,5) );
    m_bodies[146].set_orientation( quaternion_type(0.990941,-0.0309137,0.0667195,0.11238) );
    m_bodies[147].set_position( vector3_type(-1.98553,-2.14789,6.25) );
    m_bodies[147].set_orientation( quaternion_type(0.266767,-0.277306,-0.299982,0.872896) );
    m_bodies[148].set_position( vector3_type(-1.48625,2.27363,7.5) );
    m_bodies[148].set_orientation( quaternion_type(-0.976678,-0.0400053,0.061199,0.201877) );
    m_bodies[149].set_position( vector3_type(-3.10666,-3.84277,8.75) );
    m_bodies[149].set_orientation( quaternion_type(-0.142703,-0.30599,-0.378493,0.861829) );
    m_bodies[150].set_position( vector3_type(1.88397,-1.38639,10) );
    m_bodies[150].set_orientation( quaternion_type(0.839051,0.0998039,-0.0734448,0.529754) );
    m_bodies[151].set_position( vector3_type(-3.85229,3.0173,11.25) );
    m_bodies[151].set_orientation( quaternion_type(-0.991133,-0.0417237,0.0326801,0.121847) );
    m_bodies[152].set_position( vector3_type(-3.85498,2.41426,2.5) );
    m_bodies[152].set_orientation( quaternion_type(0.999922,-0.00925704,0.00579742,0.00600331) );
    m_bodies[153].set_position( vector3_type(1.13761,3.50804,3.75) );
    m_bodies[153].set_orientation( quaternion_type(0.118179,0.214778,0.662309,0.707991) );
    m_bodies[154].set_position( vector3_type(2.58419,3.4724,5) );
    m_bodies[154].set_orientation( quaternion_type(-0.432371,0.352344,0.473448,0.68173) );
    m_bodies[155].set_position( vector3_type(-2.82321,3.8435,6.25) );
    m_bodies[155].set_orientation( quaternion_type(-0.952955,-0.108851,0.148189,0.240973) );
    m_bodies[156].set_position( vector3_type(-1.23136,3.73901,7.5) );
    m_bodies[156].set_orientation( quaternion_type(0.663043,-0.108824,0.330442,0.662827) );
    m_bodies[157].set_position( vector3_type(0.440077,-1.34367,8.75) );
    m_bodies[157].set_orientation( quaternion_type(0.481483,0.0435164,-0.132867,0.865232) );
    m_bodies[158].set_position( vector3_type(1.95648,-0.900784,10) );
    m_bodies[158].set_orientation( quaternion_type(0.055532,0.190967,-0.0879231,0.976072) );
    m_bodies[159].set_position( vector3_type(-2.13202,-0.719871,11.25) );
    m_bodies[159].set_orientation( quaternion_type(-0.927322,-0.0695502,-0.0234834,0.366994) );
    m_bodies[160].set_position( vector3_type(1.4987,-1.84588,2.5) );
    m_bodies[160].set_orientation( quaternion_type(0.205101,0.425156,-0.523644,0.709206) );
    m_bodies[161].set_position( vector3_type(0.0753197,3.02878,3.75) );
    m_bodies[161].set_orientation( quaternion_type(0.95511,0.00462844,0.18612,0.23044) );
    m_bodies[162].set_position( vector3_type(-3.7517,-1.99799,5) );
    m_bodies[162].set_orientation( quaternion_type(-0.270276,-0.550406,-0.293121,0.733542) );
    m_bodies[163].set_position( vector3_type(-0.74575,-0.520157,6.25) );
    m_bodies[163].set_orientation( quaternion_type(0.930342,-0.0432982,-0.0302003,0.362874) );
    m_bodies[164].set_position( vector3_type(0.965484,-2.98044,7.5) );
    m_bodies[164].set_orientation( quaternion_type(0.959434,0.0334892,-0.103381,0.260148) );
    m_bodies[165].set_position( vector3_type(0.0836207,-2.2458,8.75) );
    m_bodies[165].set_orientation( quaternion_type(0.556741,0.00768902,-0.206504,0.804573) );
    m_bodies[166].set_position( vector3_type(-2.27705,-3.35765,10) );
    m_bodies[166].set_orientation( quaternion_type(-0.786443,-0.130328,-0.192177,0.572355) );
    m_bodies[167].set_position( vector3_type(3.63353,2.7983,11.25) );
    m_bodies[167].set_orientation( quaternion_type(-0.277561,0.287332,0.221284,0.889626) );
    m_bodies[168].set_position( vector3_type(0.314585,0.147832,2.5) );
    m_bodies[168].set_orientation( quaternion_type(-0.878364,0.0595745,0.0279956,0.473438) );
    m_bodies[169].set_position( vector3_type(0.271371,-3.33518,3.75) );
    m_bodies[169].set_orientation( quaternion_type(0.417705,0.0490585,-0.602936,0.677926) );
    m_bodies[170].set_position( vector3_type(-0.341197,0.434462,5) );
    m_bodies[170].set_orientation( quaternion_type(-0.999381,-0.00238611,0.00303834,0.0349667) );
    m_bodies[171].set_position( vector3_type(3.56932,-3.20554,6.25) );
    m_bodies[171].set_orientation( quaternion_type(0.94703,0.145485,-0.130658,0.25475) );
    m_bodies[172].set_position( vector3_type(-1.79266,-3.19529,7.5) );
    m_bodies[172].set_orientation( quaternion_type(0.242755,-0.208341,-0.371353,0.871643) );
    m_bodies[173].set_position( vector3_type(-2.79049,-0.703757,8.75) );
    m_bodies[173].set_orientation( quaternion_type(-0.11875,-0.300805,-0.0758624,0.943218) );
    m_bodies[174].set_position( vector3_type(-2.65011,0.74575,10) );
    m_bodies[174].set_orientation( quaternion_type(0.612637,-0.201942,0.0568273,0.762015) );
    m_bodies[175].set_position( vector3_type(2.89108,-0.769677,11.25) );
    m_bodies[175].set_orientation( quaternion_type(0.131877,0.246184,-0.0655402,0.95797) );
    m_bodies[176].set_position( vector3_type(1.82122,-1.2902,2.5) );
    m_bodies[176].set_orientation( quaternion_type(-0.859342,0.277911,-0.196879,0.38149) );
    m_bodies[177].set_position( vector3_type(-2.76461,-0.112918,3.75) );
    m_bodies[177].set_orientation( quaternion_type(0.86382,-0.298868,-0.0122071,0.405394) );
    m_bodies[178].set_position( vector3_type(1.10636,-0.103397,5) );
    m_bodies[178].set_orientation( quaternion_type(0.935247,0.0764634,-0.00714603,0.345564) );
    m_bodies[179].set_position( vector3_type(-2.30634,3.07077,6.25) );
    m_bodies[179].set_orientation( quaternion_type(-0.153986,-0.310653,0.413618,0.841845) );
    m_bodies[180].set_position( vector3_type(-0.151738,1.03458,7.5) );
    m_bodies[180].set_orientation( quaternion_type(0.175075,-0.0197284,0.134512,0.975124) );
    m_bodies[181].set_position( vector3_type(-2.54805,-3.62133,8.75) );
    m_bodies[181].set_orientation( quaternion_type(-0.7077,-0.183574,-0.260898,0.630392) );
    m_bodies[182].set_position( vector3_type(1.25333,-3.83935,10) );
    m_bodies[182].set_orientation( quaternion_type(0.497119,0.100836,-0.308892,0.804544) );
    m_bodies[183].set_position( vector3_type(-2.66182,1.48405,11.25) );
    m_bodies[183].set_orientation( quaternion_type(-0.0150041,-0.22835,0.127312,0.965103) );
    m_bodies[184].set_position( vector3_type(-0.503067,-0.595843,2.5) );
    m_bodies[184].set_orientation( quaternion_type(0.8801,-0.0912062,-0.108027,0.45325) );
    m_bodies[185].set_position( vector3_type(-1.27238,-0.584124,3.75) );
    m_bodies[185].set_orientation( quaternion_type(0.974316,-0.0715792,-0.0328606,0.210961) );
    m_bodies[186].set_position( vector3_type(-2.86837,0.765282,5) );
    m_bodies[186].set_orientation( quaternion_type(0.337577,-0.464322,0.123881,0.809383) );
    m_bodies[187].set_position( vector3_type(-2.54976,-3.15842,6.25) );
    m_bodies[187].set_orientation( quaternion_type(-0.81785,-0.196876,-0.243873,0.482584) );
    m_bodies[188].set_position( vector3_type(-0.508683,-1.10245,7.5) );
    m_bodies[188].set_orientation( quaternion_type(0.0149083,-0.0669452,-0.145088,0.987039) );
    m_bodies[189].set_position( vector3_type(2.69649,1.34684,8.75) );
    m_bodies[189].set_orientation( quaternion_type(0.48702,0.254478,0.127107,0.82577) );
    m_bodies[190].set_position( vector3_type(2.09882,-1.11026,10) );
    m_bodies[190].set_orientation( quaternion_type(0.239219,0.198276,-0.104887,0.944701) );
    m_bodies[191].set_position( vector3_type(0.922269,1.71795,11.25) );
    m_bodies[191].set_orientation( quaternion_type(0.284646,0.0774338,0.144239,0.944551) );
    m_bodies[192].set_position( vector3_type(3.07077,-0.954253,2.5) );
    m_bodies[192].set_orientation( quaternion_type(-0.690841,0.545084,-0.169387,0.443768) );
    m_bodies[193].set_position( vector3_type(0.757714,1.22233,3.75) );
    m_bodies[193].set_orientation( quaternion_type(0.606633,0.149981,0.241946,0.74227) );
    m_bodies[194].set_position( vector3_type(0.526261,-1.33268,5) );
    m_bodies[194].set_orientation( quaternion_type(0.32473,0.0956965,-0.242338,0.909211) );
    m_bodies[195].set_position( vector3_type(-1.08878,-0.109745,6.25) );
    m_bodies[195].set_orientation( quaternion_type(-0.926242,-0.0646789,-0.00651938,0.371282) );
    m_bodies[196].set_position( vector3_type(-1.00967,2.83395,7.5) );
    m_bodies[196].set_orientation( quaternion_type(0.975827,-0.0273065,0.0766438,0.202837) );
    m_bodies[197].set_position( vector3_type(3.07712,-2.20356,8.75) );
    m_bodies[197].set_orientation( quaternion_type(0.463655,0.28598,-0.204793,0.813203) );
    m_bodies[198].set_position( vector3_type(0.802393,0.184454,10) );
    m_bodies[198].set_orientation( quaternion_type(0.781084,0.0499346,0.0114789,0.622321) );
    m_bodies[199].set_position( vector3_type(-0.159551,3.34007,11.25) );
    m_bodies[199].set_orientation( quaternion_type(-0.00675926,-0.0135942,0.284583,0.958531) );
    m_bodies[200].set_position( vector3_type(-1.47917,1.69915,2.5) );
    m_bodies[200].set_orientation( quaternion_type(-0.474662,-0.386869,0.444403,0.653861) );
    m_bodies[201].set_position( vector3_type(3.58519,-1.47331,3.75) );
    m_bodies[201].set_orientation( quaternion_type(-0.592978,0.535277,-0.219969,0.559883) );
    m_bodies[202].set_position( vector3_type(0.330699,-3.52,5) );
    m_bodies[202].set_orientation( quaternion_type(-0.911069,0.0222629,-0.23697,0.336604) );
    m_bodies[203].set_position( vector3_type(2.39863,-2.74972,6.25) );
    m_bodies[203].set_orientation( quaternion_type(0.224203,0.322994,-0.37027,0.84161) );
    m_bodies[204].set_position( vector3_type(0.448866,0.532609,7.5) );
    m_bodies[204].set_orientation( quaternion_type(-0.843147,0.0320418,0.0380197,0.535379) );
    m_bodies[205].set_position( vector3_type(0.978668,-3.42674,8.75) );
    m_bodies[205].set_orientation( quaternion_type(0.0721803,0.103316,-0.361753,0.923716) );
    m_bodies[206].set_position( vector3_type(-1.46965,-1.45329,10) );
    m_bodies[206].set_orientation( quaternion_type(-0.0748576,-0.143519,-0.141922,0.976554) );
    m_bodies[207].set_position( vector3_type(0.539933,-2.99167,11.25) );
    m_bodies[207].set_orientation( quaternion_type(-0.765277,0.0298242,-0.16525,0.621413) );
    m_bodies[208].set_position( vector3_type(-3.92431,2.67843,2.5) );
    m_bodies[208].set_orientation( quaternion_type(0.917477,-0.290762,0.198451,0.185231) );
    m_bodies[209].set_position( vector3_type(3.71996,3.80004,3.75) );
    m_bodies[209].set_orientation( quaternion_type(-0.861395,0.29038,0.296631,0.292725) );
    m_bodies[210].set_position( vector3_type(1.83636,-3.67016,5) );
    m_bodies[210].set_orientation( quaternion_type(-0.57052,0.233154,-0.465983,0.634827) );
    m_bodies[211].set_position( vector3_type(-3.51537,1.2045,6.25) );
    m_bodies[211].set_orientation( quaternion_type(-0.76367,-0.312126,0.106947,0.554931) );
    m_bodies[212].set_position( vector3_type(-1.77996,2.93332,7.5) );
    m_bodies[212].set_orientation( quaternion_type(-0.155028,-0.213207,0.351358,0.898363) );
    m_bodies[213].set_position( vector3_type(3.03635,3.77319,8.75) );
    m_bodies[213].set_orientation( quaternion_type(0.962744,0.0820996,0.102023,0.236591) );
    m_bodies[214].set_position( vector3_type(-3.30125,-0.271371,10) );
    m_bodies[214].set_orientation( quaternion_type(0.531809,-0.265391,-0.0218158,0.80391) );
    m_bodies[215].set_position( vector3_type(-2.28193,0.631733,11.25) );
    m_bodies[215].set_orientation( quaternion_type(-0.930377,-0.0727673,0.020145,0.358746) );
    m_bodies[216].set_position( vector3_type(-1.78582,-3.45213,2.5) );
    m_bodies[216].set_orientation( quaternion_type(-0.14479,-0.382361,-0.739134,0.535273) );
    m_bodies[217].set_position( vector3_type(1.92157,0.762597,3.75) );
    m_bodies[217].set_orientation( quaternion_type(-0.909322,0.186719,0.0741015,0.364388) );
    m_bodies[218].set_position( vector3_type(2.58419,-0.405652,5) );
    m_bodies[218].set_orientation( quaternion_type(-0.97549,0.100769,-0.0158182,0.194973) );
    m_bodies[219].set_position( vector3_type(-2.00824,3.28636,6.25) );
    m_bodies[219].set_orientation( quaternion_type(-0.948619,-0.0865573,0.141645,0.269382) );
    m_bodies[220].set_position( vector3_type(2.90768,-3.77465,7.5) );
    m_bodies[220].set_orientation( quaternion_type(0.762307,0.211794,-0.274943,0.546295) );
    m_bodies[221].set_position( vector3_type(-3.86547,-3.25852,8.75) );
    m_bodies[221].set_orientation( quaternion_type(0.825338,-0.21598,-0.182067,0.488899) );
    m_bodies[222].set_position( vector3_type(-3.02658,0.732322,10) );
    m_bodies[222].set_orientation( quaternion_type(-0.672818,-0.213784,0.0517279,0.706355) );
    m_bodies[223].set_position( vector3_type(-1.67766,3.3679,11.25) );
    m_bodies[223].set_orientation( quaternion_type(0.935992,-0.0497847,0.0999425,0.333844) );
    m_bodies[224].set_position( vector3_type(-0.335337,1.12247,2.5) );
    m_bodies[224].set_orientation( quaternion_type(0.166762,-0.11976,0.400871,0.892833) );
    m_bodies[225].set_position( vector3_type(3.20017,0.385632,3.75) );
    m_bodies[225].set_orientation( quaternion_type(-0.434359,0.582925,0.0702445,0.683079) );
    m_bodies[226].set_position( vector3_type(0.364879,-1.09977,5) );
    m_bodies[226].set_orientation( quaternion_type(-0.76731,0.0455895,-0.137409,0.624721) );
    m_bodies[227].set_position( vector3_type(3.99243,-1.72137,6.25) );
    m_bodies[227].set_orientation( quaternion_type(0.242848,0.508692,-0.219326,0.796338) );
    m_bodies[228].set_position( vector3_type(1.36515,-2.15155,7.5) );
    m_bodies[228].set_orientation( quaternion_type(-0.992998,0.0203594,-0.0320875,0.111852) );
    m_bodies[229].set_position( vector3_type(3.40843,-3.92578,8.75) );
    m_bodies[229].set_orientation( quaternion_type(0.129881,0.332046,-0.382445,0.852416) );
    m_bodies[230].set_position( vector3_type(1.27067,-2.19062,10) );
    m_bodies[230].set_orientation( quaternion_type(0.988907,0.0182965,-0.0315429,0.143991) );
    m_bodies[231].set_position( vector3_type(-2.45039,-2.37519,11.25) );
    m_bodies[231].set_orientation( quaternion_type(0.191569,-0.204573,-0.198296,0.939218) );
    m_bodies[232].set_position( vector3_type(3.4055,1.46648,2.5) );
    m_bodies[232].set_orientation( quaternion_type(-0.0911002,0.758365,0.326567,0.556721) );
    m_bodies[233].set_position( vector3_type(0.815088,0.889554,3.75) );
    m_bodies[233].set_orientation( quaternion_type(-0.502599,0.178879,0.195221,0.822973) );
    m_bodies[234].set_position( vector3_type(1.21964,2.12445,5) );
    m_bodies[234].set_orientation( quaternion_type(-0.314738,0.207919,0.362167,0.852376) );
    m_bodies[235].set_position( vector3_type(-0.191778,0.215217,6.25) );
    m_bodies[235].set_orientation( quaternion_type(-0.417618,-0.0278511,0.0312549,0.907658) );
    m_bodies[236].set_position( vector3_type(-3.30393,0.491836,7.5) );
    m_bodies[236].set_orientation( quaternion_type(0.992219,-0.0501039,0.00745866,0.113737) );
    m_bodies[237].set_position( vector3_type(-3.75781,1.61711,8.75) );
    m_bodies[237].set_orientation( quaternion_type(0.77784,-0.244498,0.105216,0.569311) );
    m_bodies[238].set_position( vector3_type(3.7561,2.46236,10) );
    m_bodies[238].set_orientation( quaternion_type(-0.584919,0.277911,0.182188,0.739893) );
    m_bodies[239].set_position( vector3_type(2.84738,3.06149,11.25) );
    m_bodies[239].set_orientation( quaternion_type(0.0992127,0.236076,0.253828,0.932736) );
    m_bodies[240].set_position( vector3_type(-3.33518,-2.1059,2.5) );
    m_bodies[240].set_orientation( quaternion_type(-0.445895,-0.639254,-0.403637,0.479175) );
    m_bodies[241].set_position( vector3_type(0.276498,3.74535,3.75) );
    m_bodies[241].set_orientation( quaternion_type(0.958755,0.014808,0.200585,0.200834) );
    m_bodies[242].set_position( vector3_type(3.43968,0.393689,5) );
    m_bodies[242].set_orientation( quaternion_type(-0.966803,0.144519,0.016541,0.210077) );
    m_bodies[243].set_position( vector3_type(-0.987945,2.37886,6.25) );
    m_bodies[243].set_orientation( quaternion_type(0.0415506,-0.14602,0.351598,0.923759) );
    m_bodies[244].set_position( vector3_type(0.254036,-3.90503,7.5) );
    m_bodies[244].set_orientation( quaternion_type(-0.939192,0.0103119,-0.158514,0.304442) );
    m_bodies[245].set_position( vector3_type(-1.72893,0.465712,8.75) );
    m_bodies[245].set_orientation( quaternion_type(0.165816,-0.190901,0.0514219,0.966135) );
    m_bodies[246].set_position( vector3_type(-1.15421,1.4423,10) );
    m_bodies[246].set_orientation( quaternion_type(0.624988,-0.0886025,0.110718,0.767647) );
    m_bodies[247].set_position( vector3_type(1.91278,2.58541,11.25) );
    m_bodies[247].set_orientation( quaternion_type(0.0772475,0.162988,0.220302,0.958611) );
    m_bodies[248].set_position( vector3_type(-1.07804,-2.02997,2.5) );
    m_bodies[248].set_orientation( quaternion_type(-0.828626,-0.177705,-0.334622,0.412103) );
    m_bodies[249].set_position( vector3_type(3.34422,-2.32685,3.75) );
    m_bodies[249].set_orientation( quaternion_type(-0.652136,0.45786,-0.318571,0.513416) );

    real_type e_n = 0.1;
    m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(.4);
    default_material->normal_restitution() = e_n;
    m_configuration.set_material_library(m_library);
  }

  void setup_jamm1(real_type const & /*friction*/,real_type const & /*restitution*/)
  {
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    quaternion_type Q;
    matrix3x3_type R;
    R = OpenTissue::math::diag(value_traits::one());

    m_sphere[0].radius(1);
    m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(1,1,1));

    //m_bodies[0].attach(&m_gravity);
    m_bodies[0].set_position(vector3_type(-2.75,0,0));
    m_bodies[0].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_geometry(&m_box);
    m_bodies[0].set_fixed(true);
    m_configuration.add(&m_bodies[0]);

    //m_bodies[1].attach(&m_gravity);
    m_bodies[1].set_position(vector3_type(-0.75,0,0));
    m_bodies[1].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[1]);

    //m_bodies[2].attach(&m_gravity);
    m_bodies[2].set_position(vector3_type(0.75,0,0));
    m_bodies[2].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[2].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[2].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[2]);

    //m_bodies[3].attach(&m_gravity);
    m_bodies[3].set_position(vector3_type(2.75,0,0));
    m_bodies[3].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[3].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[3].set_geometry(&m_box);
    m_bodies[3].set_fixed(true);
    m_configuration.add(&m_bodies[3]);

    m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(.0);
    default_material->normal_restitution() = (.0);
    m_configuration.set_material_library(m_library);
  }

  void setup_jamm2(real_type const & /*friction*/,real_type const & /*restitution*/)
  {
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    quaternion_type Q;
    matrix3x3_type R;
    R = OpenTissue::math::diag(value_traits::one());

    m_sphere[0].radius(1);
    m_small_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(1,3,1));
    m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(4.5,1,1));

    m_bodies[0].set_position(vector3_type(-3.5,0,0));
    m_bodies[0].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_geometry(&m_small_box);
    m_bodies[0].set_fixed(true);
    m_configuration.add(&m_bodies[0]);

    m_bodies[1].set_position(vector3_type(3.5,0,0));
    m_bodies[1].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_geometry(&m_small_box);
    m_bodies[1].set_fixed(true);
    m_configuration.add(&m_bodies[1]);

    m_bodies[2].set_position(vector3_type(0,-4,0));
    m_bodies[2].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[2].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[2].set_geometry(&m_box);
    m_bodies[2].set_fixed(true);
    m_configuration.add(&m_bodies[2]);

    m_bodies[4].attach(&m_gravity);
    m_bodies[4].set_position(vector3_type(-1.5,-2,0));
    m_bodies[4].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[4].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[4].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[4]);

    m_bodies[5].attach(&m_gravity);
    m_bodies[5].set_position(vector3_type(1.5,-2,0));
    m_bodies[5].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[5].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[5].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[5]);

    real_type b = sqrt(value_traits::two()*value_traits::two() - 1.5*1.5);

    m_bodies[6].attach(&m_gravity);
    m_bodies[6].set_position(vector3_type(0,-2+b,0));
    m_bodies[6].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[6].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[6].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[6]);

    m_bodies[7].attach(&m_gravity);
    m_bodies[7].set_position(vector3_type(0,2-b,0));
    m_bodies[7].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[7].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[7].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[7]);

    m_bodies[8].attach(&m_gravity);
    m_bodies[8].set_position(vector3_type(-1.5,2,0));
    m_bodies[8].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[8].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[8].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[8]);

    m_bodies[9].attach(&m_gravity);
    m_bodies[9].set_position(vector3_type(1.5,2,0));
    m_bodies[9].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[9].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[9].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[9]);

    m_gravity.set_acceleration(vector3_type(0,-9.81,0));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(.0);
    default_material->normal_restitution() = (.0);
    m_configuration.set_material_library(m_library);
  }


  void setup_sphere()
  {
    m_configuration.clear();
    m_library.clear();
    m_simulator.clear();

    quaternion_type Q;
    matrix3x3_type R;
    R = OpenTissue::math::diag(value_traits::one());

    m_box.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(10,10,1));
    m_sphere[0].radius(1);

    m_bodies[0].set_position(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    Q.Rx(0.01);
    m_bodies[0].set_orientation(Q);
    m_bodies[0].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[0].set_geometry(&m_box);
    m_bodies[0].set_fixed(true);
    m_configuration.add(&m_bodies[0]);

    m_bodies[1].attach(&m_gravity);
    m_bodies[1].set_position(vector3_type(value_traits::zero(),0,2.5));
    m_bodies[1].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_bodies[1].set_geometry(&m_sphere[0]);
    m_configuration.add(&m_bodies[1]);

    m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    m_simulator.init(m_configuration);
    material_type * default_material = m_library.default_material();
    default_material->set_friction_coefficient(0.4);
    default_material->normal_restitution() = 0.1;
    m_configuration.set_material_library(m_library);
    m_timestep = 0.01;
  };


public:

  Application() { }

public:

  char const * do_get_title() const { return "Multibody Cows Demo Application"; }

  void do_display()
  {
    if(m_simulation_on || m_recording_on)
    {
      OpenTissue::utility::Timer<real_type> watch;
      watch.start();
      m_simulator.run(m_timestep);
      watch.stop();
      if(!m_statistics_on)
      {
        std::cout << "time = " << watch() << " secs.";
        std::cout << "\t|C| = " << OpenTissue::mbd::compute_contact_count(m_configuration.body_begin(),m_configuration.body_end());
        real_type min_dist;
        real_type max_dist;
        OpenTissue::mbd::compute_min_max_distance(m_configuration,min_dist,max_dist);
        std::cout << "\tmin = " << min_dist << "\tmax = " << max_dist << std::endl;
      }
      if(m_statistics_on)
      {
        if(m_cur==0)
        {
          std::cout << "allocating for stats :  " << m_max_frames << std::endl;
          m_timings.resize(m_max_frames,false);
          m_contacts.resize(m_max_frames,false);
          m_energy_kin.resize(m_configuration.size_bodies(),m_max_frames,false);
          m_energy_pot.resize(m_configuration.size_bodies(),m_max_frames,false);
        }

        if(m_cur<m_max_frames)
        {
          m_timings(m_cur) = m_simulator.time();
          m_contacts(m_cur) = OpenTissue::mbd::compute_contact_count(m_configuration.body_begin(),m_configuration.body_end());
          size_type i = 0;
          for(configuration_type::body_iterator body =  m_configuration.body_begin();body!= m_configuration.body_end();++body)
          {
            m_energy_kin(i,m_cur) = body->compute_kinetic_energy();
            vector3_type p;
            body->get_position(p);
            m_energy_pot(i,m_cur) = body->get_mass()*9.81*p(2);
            ++i;
          }
          std::cout << "frame = " << m_cur << std::endl;
        }
        else if(m_cur==m_max_frames)
        {
          using namespace OpenTissue::math::big;

          std::cout << "writting statitics to file..." << std::endl;
          std::string filename = m_prefix + ".m";
          std::ofstream m_file(filename.c_str(),std::ios::out | std::ios::app);
          static int tmp = 0;
          m_file << "T"<< tmp <<" = " << m_timings     << ";" << std::endl;
          m_file << "C"<< tmp <<" = " << m_contacts    << ";" << std::endl;
          m_file << "K"<< tmp <<" = " << m_energy_kin  << ";" <<std::endl;
          m_file << "P"<< tmp <<" = " << m_energy_pot  << ";" <<std::endl;
          ++tmp;
          m_file.flush();
          m_file.close();
        }

        ++m_cur;
      }
    }

    if(m_wireframe_on)
      for_each(m_configuration.body_begin(),m_configuration.body_end(),OpenTissue::mbd::DrawBodyFunctor<true>());
    else
      for_each(m_configuration.body_begin(),m_configuration.body_end(),OpenTissue::mbd::DrawBodyFunctor<false>());
    OpenTissue::mbd::draw_contacts(m_configuration);
    //OpenTissue::mbd::draw_penetrations(m_configuration);

    if(m_recording_on)
    {
      std::string filename = m_prefix + ".mel";
      std::ofstream mel_file;
      if(m_framecount==0)
        mel_file.open(filename.c_str(),std::ios::out);
      else
        mel_file.open(filename.c_str(),std::ios::out | std::ios::app);
      if(!mel_file.is_open())
        std::cout << "error could not open file = " << filename.c_str() << std::endl;
      if(m_framecount==0)
        mel_file << OpenTissue::mbd::mel::geometry_string(m_configuration.body_begin(),m_configuration.body_end())  << std::endl;
      if((m_framecount%m_inbetween)==0)
        mel_file << OpenTissue::mbd::mel::keyframe_string(m_configuration.body_begin(),m_configuration.body_end(),m_simulator.time())  << std::endl;
      if(m_framecount==m_max_frames)
        mel_file << OpenTissue::mbd::mel::euler_filter_string(m_configuration.body_begin(),m_configuration.body_end())  << std::endl;
      mel_file.flush();
      mel_file.close();
      std::cout << "frame = " << m_framecount << " time = " << m_simulator.time() << std::endl;
      ++m_framecount;
      if(m_framecount>m_max_frames)
        m_recording_on = false;
    }
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];
    std::cout << "choice: " << choice << " = " << (int)choice << " value "<< m_action[choice] << std::endl;
    switch(choice)
    {
    case '1':
      m_prefix = "falling_cows";
      setup_falling_cows();
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case '2':
      m_prefix = "cow_stack";
      setup_cow_stack(.01,true);
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case '3':
      m_prefix = "bowling";
      setup_bowling();
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case '4':
      m_prefix = "cow";
      setup_cow();
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case '5':
      m_prefix = "box";
      setup_box();
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case '6':
      m_prefix = "jamm1";
      setup_jamm1(0.4,0.15);
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case '7':
      m_prefix = "jamm2";
      setup_jamm2(0.4,0.4);
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case '8':
      m_prefix = "sphere";
      setup_sphere();
      m_simulator.get_stepper()->set_fraction(0.00125);
      reset_timestep();
      break;
    case 'A': 
      m_recording_on = true;  
      m_framecount = 0;
      break;
    case 'D':    m_statistics_on = ! m_statistics_on;      break;
    case 'R':    m_simulation_on = !m_simulation_on;    break;
    case 'S':
      m_simulator.run(m_timestep);
      std::cout << "|C| = " << OpenTissue::mbd::compute_contact_count(m_configuration.body_begin(),m_configuration.body_end());
      real_type min_dist;
      real_type max_dist;
      OpenTissue::mbd::compute_min_max_distance(m_configuration,min_dist,max_dist);
      std::cout << "\tmin = " << min_dist << "\tmax = " << max_dist << std::endl;
      break;
    case 'W': m_wireframe_on = !m_wireframe_on;            break;
    case '+': increment_timestep();                        break;
    case '-': decrement_timestep();                        break;
    case 'r': reset_timestep();                            break;
    default: break;
    };

    std::cout << "Dump statistics = " << m_statistics_on << std::endl;
    std::cout << "MEL recording = "   << m_recording_on  << std::endl;
    std::cout << "Run simulation = "  << m_simulation_on << std::endl;
    std::cout << "Timestep = " 
      << m_timestep 
      << " max frames = " 
      << m_max_frames 
      << " inbetween = " 
      << m_inbetween
      << std::endl;       
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu(menu);
    glutAddMenuEntry("falling cows      [1]", '1');
    glutAddMenuEntry("cow stack         [2]", '2');
    glutAddMenuEntry("bowling cows      [3]", '3');
    glutAddMenuEntry("cow               [4]", '4');
    glutAddMenuEntry("box               [5]", '5');
    glutAddMenuEntry("Jamm 1            [6]", '6');
    glutAddMenuEntry("Jamm 2            [7]", '7');
    glutAddMenuEntry("Falling and Rolling ball [8]", '8');
    glutAddMenuEntry("MEL recording     [A]", 'A');
    glutAddMenuEntry("Dump statistics   [D]", 'D');
    glutAddMenuEntry("Run simulation    [R]", 'R');
    glutAddMenuEntry("Single Step       [S]", 'S');
    glutAddMenuEntry("Wireframe         [w]", 'W');
    glutAddMenuEntry("Decrease timestep [-]", '+');
    glutAddMenuEntry("Increase timestep [+]", '-');
    glutAddMenuEntry("Reset timestep    [r]", 'r');
    glutSetMenu(main_menu);
    glutAddSubMenu("multibody", controls);
  }

  void do_init()
  {
    m_wireframe_on  = false;
    m_simulation_on = false;
    m_statistics_on = false;
    m_recording_on  = false;
    m_running_time = 30.0;

    OpenTissue::mbd::setup_default_geometry_dispatcher(m_simulator);

    reset_timestep();
  }

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
