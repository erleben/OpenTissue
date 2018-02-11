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
#include <OpenTissue/collision/bvh/bvh.h>

#include <OpenTissue/core/geometry/geometry_hybrid.h>

#include "hbvh_constructor_policy.h"

#include <vector>

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
public:

  typedef OpenTissue::math::BasicMathTypes<double,size_t>                               math_types;

  typedef math_types::vector3_type          vector3_type;
  typedef math_types::matrix3x3_type        matrix3x3_type;
  typedef math_types::real_type             real_type;
  typedef math_types::value_traits          value_traits;

  typedef OpenTissue::geometry::HybridVolume<math_types>                                hybrid_type;
  typedef OpenTissue::bvh::BoundingVolumeHierarchy<hybrid_type,int>                                         bvh_type;
  typedef OpenTissue::bvh::BVHGraph<bvh_type>                                               graph_type;
  typedef OpenTissue::bvh::BottomUpConstructor<bvh_type, HBVHConstructorPolicy<bvh_type> >  constructor_type;
  typedef OpenTissue::bvh::Volume2BVHGraph<graph_type>                                      converter_type;

  bvh_type         m_bvh;                 ///< The BVH data structure.
  graph_type       m_graph;               ///< Graph data structure used as initial input for the bottom-up constructor.
  converter_type   m_converter;           ///< A volume to graph conversion utility. Takes a set of initial volumes and creates a graph.
  constructor_type m_constructor;         ///< The bottom up constructor.

private:

  unsigned int m_depth;   //--- Controls to what depth the BVH shoud be drawn
  unsigned int m_height;  //--- Controls from what height the BVH shoud be drawn

public:

  Application()
    : m_depth(0)
    , m_height(1)
  {}

public:

  char const * do_get_title() const { return "Hybrid Bounding Volume Hierarchy Demo Application"; }

  void do_display()
  {
    bvh_type::bv_ptr_container nodes;

    OpenTissue::bvh::get_nodes_at_height(m_bvh,1,nodes);
    OpenTissue::gl::ColorPicker(0,.3,0);
    bvh_type::bv_iterator node = nodes.begin();
    bvh_type::bv_iterator end  = nodes.end();
    for (;node!= end;++node )
      OpenTissue::gl::DrawHybrid( node->volume() , false );

    OpenTissue::bvh::get_nodes_at_closest_height(m_bvh,m_height,nodes);
    OpenTissue::gl::ColorPicker(1,0,0);
    node = nodes.begin();
    end  = nodes.end();
    for (;node!= end;++node )
      OpenTissue::gl::DrawHybrid( node->volume(), true );
  }

  void do_action(unsigned char choice)
  {
    switch ( choice )
    {
    case '+':
      m_depth++;
      m_height++;
      std::cout << "depth = " 
        << m_depth
        << " height = "
        << m_height
        << std::endl;
      break;
    case '-':
      {
        if ( m_depth > 0 )
          --m_depth;
        m_height--;
        if ( m_height < 1 )
          m_height = 1;
        std::cout << "depth = " 
          << m_depth
          << " height = "
          << m_height
          << std::endl;
      }
      break;
    case 'b':
      {
        OpenTissue::geometry::Sphere<math_types>   cockpit;
        OpenTissue::geometry::Cylinder<math_types> leftWingConnector;
        OpenTissue::geometry::AABB<math_types>     leftWingBase;
        OpenTissue::geometry::OBB<math_types>      leftWingUpperBase;
        OpenTissue::geometry::OBB<math_types>      leftWingLowerBase;
        OpenTissue::geometry::Prism<math_types>    leftWingUpperRear;
        OpenTissue::geometry::Prism<math_types>    leftWingUpperFront;
        OpenTissue::geometry::Prism<math_types>    leftWingLowerRear;
        OpenTissue::geometry::Prism<math_types>    leftWingLowerFront;
        OpenTissue::geometry::Cylinder<math_types> rightWingConnector;
        OpenTissue::geometry::AABB<math_types>     rightWingBase;
        OpenTissue::geometry::OBB<math_types>      rightWingUpperBase;
        OpenTissue::geometry::OBB<math_types>      rightWingLowerBase;
        OpenTissue::geometry::Prism<math_types>    rightWingUpperRear;
        OpenTissue::geometry::Prism<math_types>    rightWingUpperFront;
        OpenTissue::geometry::Prism<math_types>    rightWingLowerRear;
        OpenTissue::geometry::Prism<math_types>    rightWingLowerFront;

        cockpit.set( vector3_type( 0, 0, 0 ), 2.0 );
        leftWingConnector.set( vector3_type( -3.5, 0, 0 ), vector3_type( -1, 0, 0 ), 4, 1 );
        leftWingBase.set( -5.5, -2, -2, -5, 2, 2 );
        rightWingConnector.set( vector3_type( 3.5, 0, 0 ), vector3_type( 1, 0, 0 ), 4, 1 );
        rightWingBase.set( 5, -2, -2, 5.5, 2, 2 );
        vector3_type p0, p1, p2;

        matrix3x3_type R;
        real_type rad = -( 35. * value_traits::pi() ) / 180.;
        R = OpenTissue::math::Rz( rad );
        leftWingUpperBase.set( vector3_type( -4.1, 3.6, 0 ), R, vector3_type( .25, 2, 2 ) );
        p0 = vector3_type( -5, 2, -2 );
        p1 = vector3_type( -5, 6, -2 );
        p2 = vector3_type( -5, 2, -6 );
        leftWingUpperRear.set( p0, p1, p2, 0.5 );
        leftWingUpperRear.xform( vector3_type( .8, -.3, 0 ), R );
        p0 = vector3_type( -5, 2, 2 );
        p1 = vector3_type( -5, 2, 14 );
        p2 = vector3_type( -5, 6, 2 );
        leftWingUpperFront.set( p0, p1, p2, 0.5 );
        leftWingUpperFront.xform( vector3_type( .8, -.3, 0 ), R );
        rad = ( 35. * value_traits::pi() ) / 180.;
        R = OpenTissue::math::Rz( rad );
        leftWingLowerBase.set( vector3_type( -4.1, -3.6, 0 ), R, vector3_type( .25, 2, 2 ) );
        p0 = vector3_type( -5, -2, -2 );
        p1 = vector3_type( -5, -2, -6 );
        p2 = vector3_type( -5, -6, -2 );
        leftWingLowerRear.set( p0, p1, p2, 0.5 );
        leftWingLowerRear.xform( vector3_type( .8, .3, 0 ), R );
        p0 = vector3_type( -5, -2, 2 );
        p1 = vector3_type( -5, -6, 2 );
        p2 = vector3_type( -5, -2, 14 );
        leftWingLowerFront.set( p0, p1, p2, 0.5 );
        leftWingLowerFront.xform( vector3_type( .8, .3, 0 ), R );
        rad = ( 35. * value_traits::pi() ) / 180.;
        R = OpenTissue::math::Rz( rad );
        rightWingUpperBase.set( vector3_type( 4.1, 3.6, 0 ), R, vector3_type( .25, 2, 2 ) );
        p0 = vector3_type( 5, 2, -2 );
        p1 = vector3_type( 5, 2, -6 );
        p2 = vector3_type( 5, 6, -2 );
        rightWingUpperRear.set( p0, p1, p2, 0.5 );
        rightWingUpperRear.xform( vector3_type( -.8, -.3, 0 ), R );
        p0 = vector3_type( 5, 2, 2 );
        p1 = vector3_type( 5, 6, 2 );
        p2 = vector3_type( 5, 2, 14 );
        rightWingUpperFront.set( p0, p1, p2, 0.5 );
        rightWingUpperFront.xform( vector3_type( -.8, -.3, 0 ), R );
        rad = -( 35. * value_traits::pi() ) / 180.;
        R = OpenTissue::math::Rz( rad );
        rightWingLowerBase.set( vector3_type( 4.1, -3.6, 0 ), R, vector3_type( .25, 2, 2 ) );
        p0 = vector3_type( 5, -2, -2 );
        p1 = vector3_type( 5, -6, -2 );
        p2 = vector3_type( 5, -2, -6 );
        rightWingLowerRear.set( p0, p1, p2, 0.5 );
        rightWingLowerRear.xform( vector3_type( -.8, .3, 0 ), R );
        p0 = vector3_type( 5, -2, 2 );
        p1 = vector3_type( 5, -2, 14 );
        p2 = vector3_type( 5, -6, 2 );
        rightWingLowerFront.set( p0, p1, p2, 0.5 );
        rightWingLowerFront.xform( vector3_type( -.8, .3, 0 ), R );

        std::vector<OpenTissue::geometry::HybridVolume<math_types> > hybrids;
        OpenTissue::geometry::HybridVolume<math_types> vol;
        vol.set( cockpit );
        hybrids.push_back( vol );
        vol.set( leftWingConnector );
        hybrids.push_back( vol );
        vol.set( leftWingBase );
        hybrids.push_back( vol );
        vol.set( leftWingUpperBase );
        hybrids.push_back( vol );
        vol.set( leftWingLowerBase );
        hybrids.push_back( vol );
        vol.set( leftWingUpperRear );
        hybrids.push_back( vol );
        vol.set( leftWingLowerRear );
        hybrids.push_back( vol );
        vol.set( leftWingUpperFront );
        hybrids.push_back( vol );
        vol.set( leftWingLowerFront );
        hybrids.push_back( vol );
        vol.set( rightWingConnector );
        hybrids.push_back( vol );
        vol.set( rightWingBase );
        hybrids.push_back( vol );
        vol.set( rightWingUpperBase );
        hybrids.push_back( vol );
        vol.set( rightWingLowerBase );
        hybrids.push_back( vol );
        vol.set( rightWingUpperRear );
        hybrids.push_back( vol );
        vol.set( rightWingLowerRear );
        hybrids.push_back( vol );
        vol.set( rightWingUpperFront );
        hybrids.push_back( vol );
        vol.set( rightWingLowerFront );
        hybrids.push_back( vol );

        m_converter.colliding( hybrids.begin(),hybrids.end(), m_graph );
        m_constructor.run( m_graph, m_bvh );
      }
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "Build Hybrid Tie Interceptor BVH  [b]", 'b' );
    glutAddMenuEntry( "Increase depth/height             [+]", '+' );
    glutAddMenuEntry( "Decrease depth/height             [-]", '-' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "HBVH", controls );
  }

  void do_init(){}

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
