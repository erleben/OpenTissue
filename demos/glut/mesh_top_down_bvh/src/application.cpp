//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/graphics/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN


#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/bvh/bvh.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/geometry/geometry_aabb.h>

#include <OpenTissue/utility/utility_timer.h>

#include "volume_refitter.h"
#include "connected_mesh_top_down_constructor.h"


class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
private:

  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;

  typedef OpenTissue::geometry::AABB<math_types>           aabb_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>       mesh_type;

  typedef mesh_type::face_type face_type;


  typedef OpenTissue::bvh::BoundingVolumeHierarchy<aabb_type,face_type*>                     bvh_type;

  typedef ConnectedMeshTopDownPolicy<bvh_type,mesh_type>  top_down_policy;
  typedef VolumeRefitter<bvh_type,mesh_type>  volume_refitter_policy;

  typedef OpenTissue::bvh::TopDownConstructor<bvh_type,top_down_policy >    constructor_type;
  typedef OpenTissue::bvh::BottomUpRefitter<volume_refitter_policy >        refitter_type;

  bvh_type         m_bvh;                 ///< The BVH data structure.
  constructor_type m_constructor;         ///< The bottom up constructor.
  refitter_type    m_refitter;            ///< Bottom up refitting of BVH.
  mesh_type        m_mesh;                ///< The original mesh, from which the BVH is builded.
  unsigned int     m_depth;               ///< Controls to what depth the BVH shoud be drawn
  unsigned int     m_height;              ///< Controls from what height the BVH shoud be drawn

public:

  Application()
    : m_depth(0)
    , m_height(1)
  { }

public:

  char const * do_get_title() const { return "Mesh Top Down BVH Demo Application"; }

  void do_display()
  {
    bvh_type::bv_ptr_container nodes;

    OpenTissue::gl::ColorPicker(0,0,.7);
    OpenTissue::gl::DrawMesh(m_mesh,GL_LINE_LOOP);

    OpenTissue::bvh::get_nodes_at_height(m_bvh,1,nodes);
    OpenTissue::gl::ColorPicker(0,.7,0);
    bvh_type::bv_iterator node = nodes.begin();
    bvh_type::bv_iterator end = nodes.end();
    for (;node!=end;++node )
      OpenTissue::gl::DrawAABB( node->volume(), false );

    OpenTissue::bvh::get_nodes_at_depth(m_bvh,m_depth,nodes);
    OpenTissue::gl::ColorPicker(1,0,0);
    node = nodes.begin();
    end = nodes.end();
    for (;node!=end;++node )
      OpenTissue::gl::DrawAABB( node->volume(), true );
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
        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/propella.obj";
        OpenTissue::mesh::obj_read( meshfile, m_mesh );

        OpenTissue::utility::Timer<real_type> watch;

        watch.start();
        m_constructor.run( m_mesh.face_begin(),m_mesh.face_end() , m_bvh );
        watch.stop();

        std::cout << "BVH construction took " << watch() << " secs." << std::endl;

        bvh_type::bv_ptr_container nodes;
        OpenTissue::bvh::get_all_nodes(m_bvh,nodes);
        assert(nodes.size()==m_bvh.size());

      }
      break;
    case 'r':
      {
        OpenTissue::mesh::spherical_bend(m_mesh,vector3_type(0.0,1.0,0.0),2.0);
        bvh_type::bv_ptr_container leaves;
        OpenTissue::bvh::get_leaf_nodes(m_bvh,leaves);
        OpenTissue::utility::Timer<real_type> watch;

        watch.start();
        m_refitter.run(leaves);
        watch.stop();

        std::cout << "BVH refit took " << watch() << " secs." << std::endl;
      }
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "Build BVH           [b]", 'b' );
    glutAddMenuEntry( "Refit BVH           [r]", 'r' );
    glutAddMenuEntry( "Decrease BVH height [-]", '-' );
    glutAddMenuEntry( "Increase BVH height [+]", '+' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "Mesh Top-Down BVH", controls );
  }

  void do_init()
  {
    this->camera().move(95);
  }

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
