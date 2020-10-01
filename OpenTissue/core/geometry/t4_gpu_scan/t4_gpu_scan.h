#ifndef OPENTISSUE_CORE_GEOMETRY_T4_GPU_SCAN_T4_GPU_SCAN_H
#define OPENTISSUE_CORE_GEOMETRY_T4_GPU_SCAN_T4_GPU_SCAN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/t4_gpu_scan/compute_obb_shell.h>

#include <OpenTissue/graphics/core/gl/gl_util.h>
#include <OpenTissue/gpu/cg/cg_util.h>
#include <OpenTissue/gpu/cg/cg_program.h>

#include <OpenTissue/gpu/texture/texture_texture2D.h>
#include <OpenTissue/gpu/texture/texture_create_texture2D.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/containers/t4mesh/t4mesh.h>

#include <OpenTissue/core/geometry/geometry_tetrahedron_z_slicer.h>
#include <OpenTissue/core/geometry/geometry_local_triangle_frame.h>

#include <boost/type_traits.hpp>  //--- needed for remove_pointer type_traits

#include<vector>

namespace OpenTissue
{

  namespace detail
  {

    template<typename real_type_ >
    class T4GPUScan
    {
    public:

      typedef real_type_          real_type;

    protected:

      struct event_type
      {
      public:

        real_type m_z_min;         ///< Lowest z-value of tetrahedron
        real_type m_z_max;         ///< Lowest z-value of tetrahedron
        int       m_idx;           ///< Tetrahedron index

      public:

        event_type( int const & idx, real_type const & z_min, real_type const & z_max)
          : m_z_min(z_min)
          , m_z_max(z_max)
          , m_idx(idx)
        {}

        bool operator <(event_type const & E) const
        {
          if(m_z_min < E.m_z_min)
            return true;
          return false;
        }
      };

      typedef std::vector<event_type>             event_container;
      typedef typename event_container::iterator  event_iterator;

    protected:

      real_type                 m_z_value;              ///< The current z-value of the sweep line.
      real_type                 m_delta_z_value;        ///< The increment in z-value when sweep line moves along z-axis.
      real_type                 m_max_z_value;          ///< The maximum z-value.
      real_type                 m_min_z_value;          ///< The minimum z-value.
      event_container           m_events;               ///< Event points for sweep line (plane) algorithm.
      event_container           m_status;               ///< The status set, i.e. the event points lying on the sweep line.
      event_iterator            m_last_event;           ///< Iterator to the last event point lying just before the sweep line. This is the place to start searching for a new status set when sweep line moves.

      real_type                 m_thickness;            ///< Narrowband value used in distance field computation

      OpenTissue::texture::texture2D_pointer         m_distance_texture[2];  ///< The texture used as a data array

      gl::FramebufferObject     m_fbo[2];               ///< The framebuffer object used for rendering to the texture
      gl::renderbuffer_pointer  m_rb_depth[2];          ///< Optional: The renderbuffer object used for depth

      cg::Program                 m_fragment_program;     ///< the fragment program used

      // OpenGL state storage
      GLint                     m_state_current_drawbuf;
      GLint                     m_state_viewport[4];            ///< Temporary storage for original viewport settings.
      float                     m_state_color_clear_value[4];   ///< Temporary storage for original clear color.
      bool                      m_state_depth_test;
      GLint                     m_state_depth_func;
      float                     m_state_depth_clear_value;

    protected:

      template<typename grid_type>
      void set_gl_state(grid_type const & phi)
      {
        gl::gl_check_errors("set_gl_state: called");

        glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);      // Draw into the first texture
        gl::gl_check_errors("set_gl_state: switched to color0 attachement");

        // Hummmmm.........
        //  setup model view matrix for vertex program
        glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);
        glMatrixMode( GL_PROJECTION );
        glLoadIdentity();
        GLdouble left   = phi.min_coord(0) - phi.dx()/2;
        GLdouble right  = phi.max_coord(0) + phi.dx()/2;
        GLdouble bottom = phi.min_coord(1) - phi.dy()/2;
        GLdouble top    = phi.max_coord(1) + phi.dy()/2;
        gluOrtho2D( left, right, bottom, top);
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();

        glViewport(0,0,phi.I(),phi.J());//--- KE is hakcing away!!! It seems to fix the wird alignment problems!!!
        gl::gl_check_errors("set_gl_state: proj and modelview setup");

        glClearColor( phi.unused(), 0, 0, 0 ); // A float buffer can be cleared with any value, including our grid.unused.
        glEnable( GL_DEPTH_TEST );
        glDepthFunc( GL_LESS );
        glClearDepth( 1.0 );  // Clear to maximum clamped value.
        gl::gl_check_errors("set_gl_state: clear and depth setup");
      }

      /**
      * Setup the FrameBufferObject stuff.
      */
      template<typename grid_type>
      void init_fbo(grid_type const & phi)
      {
        gl::gl_check_errors("init_fbo: called");

        for(int i=0; i<2; ++i)
        {
          // initialize the texture
          m_distance_texture[i] = OpenTissue::texture::create_float_texture_rectangle( phi.I(), phi.J(), 1, 0 );

          m_fbo[i].bind(); // Bind framebuffer object.
          m_fbo[i].attach_texture(GL_COLOR_ATTACHMENT0_EXT, m_distance_texture[i] );

          // Optional: initialize depth renderbuffer
          m_rb_depth[i] = gl::create_depth_buffer( phi.I(), phi.J() );
          m_fbo[i].attach_render_buffer( GL_DEPTH_ATTACHMENT_EXT, m_rb_depth[i] );
          gl::gl_check_errors("init_fbo: renderbuffer attached");

          // Validate the FBO after attaching textures and render buffers
          m_fbo[i].is_valid();
          gl::gl_check_errors("init_fbo: validated fbo");

          set_gl_state(phi);
        }
        // Disable FBO rendering for now...
        gl::FramebufferObject::disable();
        gl::gl_check_errors("init_fbo: done configuring fbo");
      }

      /**
      * Disable fbo related stuff
      */
      void disable_fbo()
      {
        gl::gl_check_errors("disable_fbo: called");

        gl::FramebufferObject::disable();
        gl::gl_check_errors("disable_fbo: fbo disabled");
      }

      /**
      * Stores the current OpenGl state
      */
      void store_gl_state()
      {
        gl::gl_check_errors("store_gl_state: called");

        glGetIntegerv(GL_DRAW_BUFFER, &m_state_current_drawbuf); // Save the current Draw buffer
        gl::gl_check_errors("store_gl_state: stored current drawbuffer");

        // store the window viewport dimensions so we can reset them,
        // and set the viewport to the dimensions of our texture
        glGetIntegerv(GL_VIEWPORT, m_state_viewport);
        gl::gl_check_errors("store_gl_state: stored current viewport");

        glGetFloatv(GL_COLOR_CLEAR_VALUE, m_state_color_clear_value); // store current clear color
        m_state_depth_test = ( glIsEnabled(GL_DEPTH_TEST) != 0 );
        glGetIntegerv(GL_DEPTH_FUNC, &m_state_depth_func);
        glGetFloatv(GL_DEPTH_CLEAR_VALUE,  &m_state_depth_clear_value);
        gl::gl_check_errors("store_gl_state: current color and depth state stored");

        glMatrixMode( GL_PROJECTION );
        glPushMatrix();
        glMatrixMode( GL_MODELVIEW );
        glPushMatrix();
        gl::gl_check_errors("store_gl_state: current projection and modelview matrices stored");
      }
      /**
      * Restores the OpenGl state
      */
      void restore_gl_state()
      {
        gl::gl_check_errors("restore_gl_state: called");

        glMatrixMode( GL_PROJECTION );
        glPopMatrix();
        glMatrixMode( GL_MODELVIEW );
        glPopMatrix();
        gl::gl_check_errors("restore_gl_state: projection and modelview matrices restored");

        glClearDepth( m_state_depth_clear_value );
        glDepthFunc( m_state_depth_func );
        if ( !m_state_depth_test )
          glDisable( GL_DEPTH_TEST );
        glClearColor( m_state_color_clear_value[0],m_state_color_clear_value[1],m_state_color_clear_value[2],m_state_color_clear_value[3] );
        gl::gl_check_errors("restore_gl_state: depth and clear color restored");

        glDrawBuffer(m_state_current_drawbuf);
        gl::gl_check_errors("restore_gl_state: back to current drawbuf");

        glViewport(m_state_viewport[0], m_state_viewport[1], m_state_viewport[2], m_state_viewport[3]);
        gl::gl_check_errors("restore_gl_state: viewport restored");
      }

      template<typename grid_type>
      bool init_rendering(grid_type const & phi, bool use_unsigned)
      {
        gl::gl_check_errors("init_rendering: called");
        if(!OpenTissue::gl::is_framebuffer_object_supported())
          return false;
        if(!OpenTissue::gl::is_float_texture_supported())
          return false;
        if(!OpenTissue::cg::startup())
          return false;
        store_gl_state();
        std::string filename;
        if(use_unsigned)
        {
          filename = opentissue_path + "/OpenTissue/core/geometry/t4_gpu_scan/fp_t4gpu_unsigned.cg";
        }
        else
        {
          filename = opentissue_path + "/OpenTissue/core/geometry/t4_gpu_scan/fp_t4gpu_faster.cg";
        }
        m_fragment_program.load_from_file(cg::Program::fragment_program, filename);
        gl::gl_check_errors("init_fragment_program: program loaded");
        init_fbo(phi);
        m_fragment_program.enable();
        return true;
      }

      void cleanup_rendering()
      {
        gl::gl_check_errors("cleanup_rendering: called");

        disable_fbo();
        m_fragment_program.disable();
        OpenTissue::cg::shutdown();
        restore_gl_state();
      }

      /**
      * Initialization.
      *
      * Sets up an event-list for the sweep-line algorithm. Also sets up
      * starting and ending positions along the sweep-line as well as the
      * increment along the sweep-line.
      *
      * @param  shell    A tetrahedron mesh, representing the shell layer that
      *                  should be scan-converted into a signed distance grid.
      * @param points    The coordinates of the vertices in the shell mesh. That
      *                  is the i'th entry holds the coordinate of the i'th vertex.
      * @param phi       The grid that is supposed to hold the signed distance grid
      *                  upon completion of the algorithm.
      */
      template<typename surface_mesh, typename point_container,typename volume_mesh, typename grid_type>
      void init(
        surface_mesh /*const*/ & surface  //--- we are messing with tags, otherwise this should be const!!!
        , volume_mesh const & shell
        , point_container const & points
        , grid_type const & phi
        , point_container & edge_normals
        ,  bool use_unsigned
        )
      {
        //--- starting and ending positions of scan-line algorithm
        m_min_z_value = phi.min_coord(2);
        m_max_z_value = phi.max_coord(2);
        m_z_value = m_min_z_value;
        //--- increment to jump along scan-line
        m_delta_z_value = phi.dz();

        //--- setting up the event list
        m_status.clear();
        m_events.clear();

        typename volume_mesh::tetrahedron_iterator begin = shell.tetrahedron_begin();
        typename volume_mesh::tetrahedron_iterator end = shell.tetrahedron_end();
        typename volume_mesh::tetrahedron_iterator T;
        for (T = begin; T != end; ++T)
        {
          real_type z_min =  std::min( points[T->i()->idx()](2), std::min( points[T->j()->idx()](2), std::min( points[T->k()->idx()](2) , points[T->m()->idx()](2) )));
          real_type z_max =  std::max( points[T->i()->idx()](2), std::max( points[T->j()->idx()](2), std::max( points[T->k()->idx()](2) , points[T->m()->idx()](2) )));
          m_events.push_back(event_type(T->idx(),z_min,z_max));
        }
        std::sort( m_events.begin(), m_events.end() );
        m_last_event = m_events.begin();

if(!use_unsigned)
{

        point_container face_normals;
        face_normals.resize(surface.size_faces());
        typename surface_mesh::face_iterator f_end = surface.face_end();
        typename surface_mesh::face_iterator f     = surface.face_begin();
        for(unsigned int i = 0;f!=f_end;++f,++i)
        {
          f->m_tag = i;
          compute_face_normal(*f, face_normals[i]);
        }

        edge_normals.resize(surface.size_edges());
        typename surface_mesh::edge_iterator e_end = surface.edge_end();
        typename surface_mesh::edge_iterator e     = surface.edge_begin();
        for(unsigned int i = 0;e!=e_end;++e,++i)
        {
          e->m_tag = i;
          edge_normals[i].clear();
          typename surface_mesh::face_type * f0=0,*f1=0;
          if(! e->get_halfedge0_iterator()->get_face_handle().is_null())
          {
            f0 = &(*e->get_halfedge0_iterator()->get_face_iterator());
            edge_normals[i] += face_normals[f0->m_tag];
          }
          if(! e->get_halfedge1_iterator()->get_face_handle().is_null())
          {
            f1 = &(*e->get_halfedge1_iterator()->get_face_iterator());
            edge_normals[i] += face_normals[f1->m_tag];
          }
          edge_normals[i] = unit( edge_normals[i] );
        }
}
      }

      /**
      * Update Status Set.
      *
      *  That is find the new set of event points corresponding to
      * tetrahedra which overlaps with the sweep-line.
      *
      *  @param z    The new position of the sweep-line.
      */
      void update_status_set(real_type const & z)
      {
        if( m_events.empty() )
          return;
        m_status.clear();

        //--- We build up a new status set from scratch. Since we know that event
        //--- points are keept sorted, we can start searching for event points
        //--- crossing the sweep-line at the last position.
        //--- We stop the searching as soon as we see an event point lying
        //--- completely on unprocessed side of the sweep line.

        event_iterator begin   = m_last_event;
        event_iterator end     = m_events.end();
        event_iterator event;
        for ( event=begin; event!=end; ++event )
        {
          if( event->m_z_min <= z && z <= event->m_z_max )
            m_status.push_back( (*event) );
          if( event->m_z_min > z )
            break;
        }
        for ( ; m_last_event!=event && m_last_event->m_z_max < z; ++m_last_event )
          ;
      }

      /**
      *  Draw Slice.
      *  This method intersects all tetrahedra in the status set with the plane
      *  that corresponds to the current position of the sweep-line. The cross
      *  sections are then rendered.
      *
      * @param shell     A tetrahedron mesh, representing the shell layer that
      *                  should be scan-converted into a signed distance grid.
      * @param lut       Lookup Table. This table holds a correspondence map
      *                  between tetrahedra and the polygonal triangular faces
      *                  from which they originate.
      *                  That is one can lookup the triangular face lying closest
      *                  to all points inside a tetrahedron than to any other
      *                  triangular faces!!!
      * @param points    The coordinates of the vertices in the shell mesh. That
      *                  is the i'th entry holds the coordinate of the i'th vertex.
      * @param z         The current position of the sweep-line.
      */
      template< typename volume_mesh, typename lut_container, typename point_container >
      void draw_slice(
        volume_mesh const & shell
        , lut_container /*const*/ &  lut
        , point_container const & points
        , real_type const & z
        , point_container & edge_normals
        )
      {
        typedef typename lut_container::value_type                  face_ptr_type;
        typedef typename boost::remove_pointer<face_ptr_type>::type face_type;

        typedef typename face_type::mesh_type                   surface_mesh;
        typedef typename surface_mesh::face_vertex_circulator   face_vertex_circulator;

        typedef typename surface_mesh::math_types               math_types;
        typedef typename math_types::vector3_type               vector3_type;


        gl::gl_check_errors("draw_slice: called");
        static geometry::LocalTriangleFrame<vector3_type> local_frame;
        static vector3_type slice[4];

        event_iterator begin = m_status.begin();
        event_iterator end   = m_status.end();
        event_iterator event;

        typename volume_mesh::tetrahedron_iterator T;
        typename volume_mesh::node_iterator i;
        typename volume_mesh::node_iterator j;
        typename volume_mesh::node_iterator k;
        typename volume_mesh::node_iterator m;
        typename surface_mesh::face_type * face;
        unsigned int cnt;
        real_type e10x;
        real_type e10y;
        real_type e21x;
        real_type e21y;
        vector3_type nv0;
        vector3_type nv1;
        vector3_type nv2;
        vector3_type ne0;
        vector3_type ne1;
        vector3_type ne2;
        vector3_type local;
        for ( event = begin; event!=end; ++event )
        {
          T = shell.tetrahedron(event->m_idx);
          i = T->i();
          j = T->j();
          k = T->k();
          m = T->m();

          face = lut[event->m_idx];
          assert(valency(*face)==3 || !"Only triangular faces are supported!");

          typename surface_mesh::face_vertex_circulator v(*face);
          vector3_type & p0 = v->m_coord;  ++v;
          vector3_type & p1 = v->m_coord;  ++v;
          vector3_type & p2 = v->m_coord;

          local_frame.init(p0,p1,p2);

          OpenTissue::geometry::ZTetrahedronSlicer<vector3_type> slicer(
            points[i->idx()]
          , points[j->idx()]
          , points[k->idx()]
          , points[m->idx()]
          );

          cnt = slicer.intersect(z,slice);

          e10x = slice[1](0)-slice[0](0);
          e10y = slice[1](1)-slice[0](1);
          e21x = slice[2](0)-slice[1](0);
          e21y = slice[2](1)-slice[1](1);
          bool flip = false;
          if ( (e10x*e21y-e21x*e10y)<=0)
            flip = true;

          {
            typename surface_mesh::face_halfedge_circulator nh(*face);
            nv0 = (nh->get_origin_iterator()->m_normal);
            ne0 = edge_normals[nh->get_edge_iterator()->m_tag];
            ++nh;
            nv1 = (nh->get_origin_iterator()->m_normal);
            ne1 = edge_normals[nh->get_edge_iterator()->m_tag];
            ++nh;
            nv2 = (nh->get_origin_iterator()->m_normal);
            ne2 = edge_normals[nh->get_edge_iterator()->m_tag];
            local_frame.compute_local_normals(nv0,nv1,nv2,ne0,ne1,ne2);
          }

          glBegin(GL_POLYGON);
          for(unsigned int i=0;i<cnt;++i)
          {
            unsigned int j = i;
            if(flip)
              j = cnt - 1 - i;
            local = local_frame.get_local_coord(slice[j]);
            glNormal3f( 0, 0, 1 );
            glMultiTexCoord3f( GL_TEXTURE0, local(0), local(1), local(2) );
            glMultiTexCoord3f( GL_TEXTURE1, local_frame.a(), local_frame.b(), local_frame.h() );
            glMultiTexCoord3f( GL_TEXTURE2, local_frame.nv0()(0), local_frame.nv0()(1), local_frame.nv0()(2) );
            glMultiTexCoord3f( GL_TEXTURE3, local_frame.nv1()(0), local_frame.nv1()(1), local_frame.nv1()(2) );
            glMultiTexCoord3f( GL_TEXTURE4, local_frame.nv2()(0), local_frame.nv2()(1), local_frame.nv2()(2) );
            glMultiTexCoord3f( GL_TEXTURE5, local_frame.ne0()(0), local_frame.ne0()(1), local_frame.ne0()(2) );
            glMultiTexCoord3f( GL_TEXTURE6, local_frame.ne1()(0), local_frame.ne1()(1), local_frame.ne1()(2) );
            glMultiTexCoord3f( GL_TEXTURE7, local_frame.ne2()(0), local_frame.ne2()(1), local_frame.ne2()(2) );
            glVertex3f( slice[j](0), slice[j](1), slice[j](2) );
          }
          glEnd();
          gl::gl_check_errors("draw_slice: polygon has been drawn");
        }
      }


      template< typename volume_mesh, typename lut_container, typename point_container >
      void draw_slice_unsigned(
        volume_mesh const & shell
        , lut_container /*const*/ &  lut
        , point_container const & points
        , real_type const & z
        )
      {
        typedef typename lut_container::value_type                  face_ptr_type;
        typedef typename boost::remove_pointer<face_ptr_type>::type face_type;

        typedef typename face_type::mesh_type                   surface_mesh;
        typedef typename surface_mesh::face_vertex_circulator   face_vertex_circulator;

        typedef typename surface_mesh::math_types               math_types;
        typedef typename math_types::vector3_type               vector3_type;


        gl::gl_check_errors("draw_slice_unsigned: called");
        static geometry::LocalTriangleFrame<vector3_type> local_frame;
        static vector3_type slice[4];

        event_iterator begin = m_status.begin();
        event_iterator end   = m_status.end();
        event_iterator event;

        typename volume_mesh::tetrahedron_iterator T;
        typename volume_mesh::node_iterator i;
        typename volume_mesh::node_iterator j;
        typename volume_mesh::node_iterator k;
        typename volume_mesh::node_iterator m;
        typename surface_mesh::face_type * face;
        unsigned int cnt;
        real_type e10x;
        real_type e10y;
        real_type e21x;
        real_type e21y;
        vector3_type local;
        for ( event = begin; event!=end; ++event )
        {
          T = shell.tetrahedron(event->m_idx);
          i = T->i();
          j = T->j();
          k = T->k();
          m = T->m();

          face = lut[event->m_idx];
          assert(valency(*face)==3 || !"Only triangular faces are supported!");

          typename surface_mesh::face_vertex_circulator v(*face);
          vector3_type & p0 = v->m_coord;  ++v;
          vector3_type & p1 = v->m_coord;  ++v;
          vector3_type & p2 = v->m_coord;

          local_frame.init(p0,p1,p2);

          OpenTissue::geometry::ZTetrahedronSlicer<vector3_type> slicer(
            points[i->idx()]
          , points[j->idx()]
          , points[k->idx()]
          , points[m->idx()]
          );

          cnt = slicer.intersect(z,slice);

          e10x = slice[1](0)-slice[0](0);
          e10y = slice[1](1)-slice[0](1);
          e21x = slice[2](0)-slice[1](0);
          e21y = slice[2](1)-slice[1](1);
          bool flip = false;
          if ( (e10x*e21y-e21x*e10y)<=0)
            flip = true;


          glBegin(GL_POLYGON);
          for(unsigned int i=0;i<cnt;++i)
          {
            unsigned int j = i;
            if(flip)
              j = cnt - 1 - i;
            local = local_frame.get_local_coord(slice[j]);
            glNormal3f( 0, 0, 1 );
            glMultiTexCoord3f( GL_TEXTURE0, local(0), local(1), local(2) );
            glMultiTexCoord3f( GL_TEXTURE1, local_frame.a(), local_frame.b(), local_frame.h() );
            glVertex3f( slice[j](0), slice[j](1), slice[j](2) );
          }
          glEnd();
          gl::gl_check_errors("draw_slice_unsigned: polygon has been drawn");
        }
      }


    public:

      /**
      * Run Tetrahedron Scan Conversion Algorithm.
      *
      * @param  shell    A tetrahedron mesh, representing the shell layer that
      *                  should be scan-converted into a signed distance grid.
      * @param  lut      Lookup Table. This table holds a correspondence map
      *                  between tetrahedra and the polygonal triangular faces
      *                  from which they originate.
      *                  That is one can lookup the triangular face lying closest
      *                  to all points inside a tetrahedron than to any other
      *                  triangular faces!!!
      * @param points    The coordinates of the vertices in the shell mesh. That
      *                  is the i'th entry holds the coordinate of the i'th vertex.
      * @param phi       The grid that is supposed to hold the signed distance grid
      *                  upon completion of the algorithm.
      *
      * @param use_unsigned   If this boolean flag is set to true then the unsigned version of the algorithm is used.
      *
      * @return          If computation succeded then the return value is true otherwise it is false.
      */
      template<typename surface_mesh,typename volume_mesh,typename lut_container, typename point_container,typename grid_type>
      bool run(
        surface_mesh /*const*/ & surface //--- we are messing with tags!!!
        , volume_mesh const & shell
        , lut_container /*const*/ &  lut
        , point_container const & points
        , real_type const & thickness
        , grid_type & phi
        , bool use_unsigned
        )
      {
        point_container edge_normals;
        
        m_thickness = thickness;

        init(surface,shell,points,phi,edge_normals,use_unsigned);

        if(!init_rendering(phi,use_unsigned))
          return false;
        
        m_fragment_program.set_float_param( "narrowband", m_thickness );
        
        update_status_set(m_z_value);
        unsigned int current = 0;
        gl::gl_check_errors("init_rendering: fbo bound");
        OpenTissue::gl::ColorPicker(1.0,0.0,0.0);
        for(unsigned int k=0; k<phi.K(); ++k)
        {
          gl::gl_check_errors("run: init loop");
          unsigned int next = (current+1)%2;
          m_fbo[current].bind();
          glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
          gl::gl_check_errors("run: buffer cleared");

          if(use_unsigned)
            draw_slice_unsigned(shell, lut, points, m_z_value);
          else         
            draw_slice(shell, lut, points, m_z_value,edge_normals);

          // read_back z-buffer into grid data structure
          float * zplane = phi.data() + k * phi.I()*phi.J();
          glReadPixels( 0, 0, phi.I(), phi.J(), GL_RED, GL_FLOAT, zplane );
          gl::gl_check_errors("run: texture readback");
          m_z_value += m_delta_z_value;
          update_status_set(m_z_value);
          current = next;
        }
        cleanup_rendering();
        return true;
      }
    };

  } // namespace detail

  
  struct t4_gpu_unsigned {};
  struct t4_gpu_signed {};

  /**
  * Default GPU Distance Field Computation.
  * This will compue a signed distance field by default.
  *
  * @param mesh             A surface mesh, used to generate the
  *                         distance field from.
  * @param thickness        An offset parameter, describing the extent
  *                         of the generated distance field along vertex normals
  *                         (not the same as face normals!).
  * @param phi              Upon return holds the generated distance field.
  * @return                 If computation succeded then the return value is true otherwise it is false.
  */
  template<typename surface_mesh, typename grid_type>
  inline bool t4_gpu_scan(
    surface_mesh /*const*/ & surface
    , double const & thickness
    , grid_type & phi
    )
  {
    return t4_gpu_scan( surface, thickness, phi, t4_gpu_signed() );
  }


  /**
  * Signed GPU Distance Field Computation.
  *
  * @param unsigned      A dispatcher tag used to select the unsigned version.
  */
  template<typename surface_mesh, typename grid_type>
  inline bool t4_gpu_scan(
    surface_mesh /*const*/ & surface
    , double const & thickness
    , grid_type & phi
    , t4_gpu_signed const & /*tag*/
    )
  {
    typedef double                                              real_type;
    typedef OpenTissue::math::Vector3<real_type>                vector3_type;
    typedef OpenTissue::t4mesh::T4Mesh<>                        volume_mesh;
    typedef std::vector< typename surface_mesh::face_type * >   lut_container;
    typedef  std::vector< vector3_type >                        point_container;

    point_container  points;
    volume_mesh      shell;
    lut_container    lut;

    detail::T4GPUScan<real_type> calculator;

    OpenTissue::mesh::compute_angle_weighted_vertex_normals(surface);

    detail::compute_obb_shell(surface,thickness,shell,points,lut);

    return calculator.run(surface,shell,lut,points,thickness,phi,false);
  }

  /**
  * Unsigned GPU Distance Field Computation.
  *
  * @param unsigned      A dispatcher tag used to select the unsigned version.
  */
  template<typename surface_mesh, typename grid_type>
  inline bool t4_gpu_scan(
    surface_mesh /*const*/ & surface
    , double const & thickness
    , grid_type & phi
    , t4_gpu_unsigned const & /*tag*/
    )
  {
    typedef double                                              real_type;
    typedef OpenTissue::math::Vector3<real_type>                      vector3_type;
    typedef OpenTissue::t4mesh::T4Mesh<>                        volume_mesh;
    typedef std::vector< typename surface_mesh::face_type * >   lut_container;
    typedef  std::vector< vector3_type >                        point_container;

    point_container  points;
    volume_mesh      shell;
    lut_container    lut;

    detail::T4GPUScan<real_type> calculator;
    detail::compute_obb_shell(surface,thickness,shell,points,lut);
    return calculator.run(surface,shell,lut,points,thickness,phi,true);
  }





} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_T4_GPU_SCAN_T4_GPU_SCAN_H
#endif
