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


#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/math/interpolation/interpolation_variational_interpolator.h>

#include <vector>

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
public:


  typedef OpenTissue::math::BasicMathTypes<double,size_t>   math_types;
  typedef math_types::real_type                             real_type;
  typedef math_types::vector3_type                          vector3_type;
  typedef math_types::matrix3x3_type                        matrix3x3_type;
  typedef float                                             value_type;

  typedef OpenTissue::grid::Grid<value_type,math_types>            phi_type;
  typedef OpenTissue::trimesh::TriMesh<>                    mesh_type;


protected:

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.
  
  static unsigned char const E_KEY = 'E';

  mesh_type                   m_isosurface;
  phi_type                    m_phi;

  std::vector<vector3_type>   m_points;
  std::vector<vector3_type>   m_normals;

public:

  Application()   {  }

public:

  char const * do_get_title() const { return "Variational Interpolator Demo Application"; }

  void do_display()
  {
    if(m_action[E_KEY])
    {
      OpenTissue::gl::ColorPicker(0.8,0.4,0.8);
      OpenTissue::gl::DrawMesh(m_isosurface);
    }
    size_t N =  m_points.size();
    for(size_t i=0;i<N;++i)
    {
      OpenTissue::gl::ColorPicker(0.1,0.4,0.8);
      OpenTissue::gl::DrawPoint(m_points[i], 0.05);
      OpenTissue::gl::ColorPicker(0.8,0.8,0.1);
      OpenTissue::gl::DrawVector(m_points[i], .25*m_normals[i], 0.75, false);
    }
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];
    switch ( choice )
    {
    case 'v'://interpolate
      {
        m_phi.create( vector3_type(-2,-2,-2), vector3_type(2,2,2), 64,64,64);
        OpenTissue::interpolation::detail::ImplicitFunction<vector3_type> F = OpenTissue::interpolation::variational_interpolator(m_points.begin(),m_points.end(),m_normals.begin(),m_normals.end());
        for(unsigned int k=0;k<64;++k)
          for(unsigned int j=0;j<64;++j)
            for(unsigned int i=0;i<64;++i)
            {
              vector3_type coord;
              OpenTissue::grid::idx2coord(m_phi,i,j,k,coord);
              m_phi(i,j,k) = F(coord);
            }
      }
      break;
    case 'e': //--- extract zero level set surface of currently shown map
      {
        OpenTissue::mesh::smooth_isosurface(m_phi,0.0,m_isosurface);
        OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_isosurface);
      }
      break;
    case 'E': //--- Toggle show/hide extracted level set surface
      {
      }
      break;
    case 'w': //--- write extracted zero level set surface to mesh file
      {
        OpenTissue::mesh::obj_write("test.obj",m_isosurface);
      }
      break;
    case 'i': //--- initialize
      {
        init();
      }
      break;
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry("Variational Interpolation             [v]", 'v' );
    glutAddMenuEntry("Extract zero-level-isosurface         [e]", 'e' );
    glutAddMenuEntry("Toggle show/hide iso-surface          [E]", 'E' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "interpolator", controls );
  }

  void do_init()
  {
    this->camera().move(90);

    m_points.resize(14);
    m_normals.resize(14);

    m_points[0] = vector3_type(  0.008337971754, -0.393710464239,  0.163935899734 );
    m_points[1] = vector3_type( -0.162570104003, -0.087559260428,  0.123178444803 );
    m_points[2] = vector3_type(  0.023525221273, -0.075225926936,  0.154702857137 );
    m_points[3] = vector3_type(  0.183924108744, -0.116216018796,  0.053398381919 );
    m_points[4] = vector3_type( -0.175580918789,  0.205400660634,  0.099069811404 );
    m_points[5] = vector3_type(  0.021648591384,  0.204933568835,  0.082646958530 );
    m_points[6] = vector3_type(  0.196496471763,  0.156683325768,  0.006553138141 );
    m_points[7] = vector3_type( -0.166723892093,  0.226445302367, -0.000685797946 );
    m_points[8] = vector3_type(  0.000629404269,  0.224662646651, -0.026558911428 );
    m_points[9] = vector3_type(  0.178584307432,  0.187080129981, -0.072855211794 );
    m_points[10] = vector3_type( -0.168895393610, -0.070832192898, -0.068285755813 );
    m_points[11] = vector3_type( -0.014609432779, -0.060379069299, -0.113564901054 );
    m_points[12] = vector3_type(  0.135422587395, -0.095618382096, -0.112379454076 );
    m_points[13] = vector3_type( -0.060189183801, -0.305664300919, -0.289155662060 );


    m_normals[0] = vector3_type(  0.121994487941, -0.001839586534,  0.992529094219 );
    m_normals[1] = vector3_type( -0.624556899071,  0.039056390524,  0.780002057552 ); 
    m_normals[2] = vector3_type(  0.247424483299,  0.166081532836,  0.954566955566 );
    m_normals[3] = vector3_type(  0.686707258224,  0.162923291326,  0.708441317081 ); 
    m_normals[4] = vector3_type( -0.427823156118,  0.428714066744,  0.795720815659 ); 
    m_normals[5] = vector3_type(  0.273757755756,  0.577548027039,  0.769087076187 ); 
    m_normals[6] = vector3_type(  0.812700212002,  0.348244249821,  0.467166215181 ); 
    m_normals[7] = vector3_type( -0.102822534740,  0.715928316116, -0.690560698509 ); 
    m_normals[8] = vector3_type( -0.046586330980,  0.744266688824, -0.666255831718 ); 
    m_normals[9] = vector3_type(  0.321366369724,  0.534169614315, -0.781912028790 ); 
    m_normals[10] = vector3_type( -0.347756743431,  0.503106296062, -0.791169643402 ); 
    m_normals[11] = vector3_type( -0.098281666636,  0.665029644966, -0.740321755409 ); 
    m_normals[12] = vector3_type(  0.252319008112,  0.340096980333, -0.905907928944 ); 
    m_normals[13] = vector3_type( -0.021462006494,  0.442783534527, -0.896371662617 ); 
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
