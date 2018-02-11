#ifndef OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_SURFACE_MESH_H
#define OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_SURFACE_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/constraints/psys_stick.h>
#include <OpenTissue/dynamics/psys/forces/psys_spring.h>
#include <OpenTissue/dynamics/psys/mass_spring_system/psys_mass_spring_system.h>

#include <list>
#include <map>

namespace OpenTissue
{
  namespace psys
  {

    /**
    * Surface Mesh.
    *
    * Takes a mesh and builds a particle system from it.
    */
    template<
        typename types
      , typename integrator_policy
    >
    class SurfaceMesh : public MassSpringSystem<types,integrator_policy>
    {
    public:

      typedef MassSpringSystem<types,integrator_policy> base_class;

    public:

      typedef typename types::math_types         math_types;
      typedef typename math_types::real_type     real_type;
      typedef typename math_types::vector3_type  vector3_type;
      typedef typename types::particle_type      particle_type;
      typedef Stick<types>                   stick_type;
      typedef Spring<types>                  spring_type;
      typedef typename types::coupling_type      coupling_type;
      typedef typename types::mesh_type          mesh_type;

    protected:

      typedef typename mesh_type::vertex_type                          vertex_type;
      typedef typename mesh_type::vertex_iterator                      vertex_iterator;
      typedef typename mesh_type::vertex_halfedge_circulator           vertex_halfedge_circulator;

      typedef std::list<stick_type>                           stick_container;
      typedef std::list<stick_type*>                          stick_ptr_container;
      typedef std::map<particle_type*, stick_ptr_container >  stick_lut_type;

      typedef std::list<spring_type>                          spring_container;
      typedef std::list<spring_type*>                         spring_ptr_container;
      typedef std::map<particle_type*, spring_ptr_container > spring_lut_type;

    public:

      coupling_type       m_coupling;             ///< Internal data structure used to find correspond particle of vertex.
      int                 m_rigidty;              ///< Constant used to determine how rigid a surface should be.
      stick_container     m_sticks;               ///< Internal data structure used to store all stick constraints.
      stick_lut_type      m_stick_lut;            ///< Internal datas tructure to record stick connections.
      spring_container    m_springs;              ///< Internal data structure used to store all spring constraints.
      spring_lut_type     m_spring_lut;           ///< Internal datas tructure to record spring connections.

    public:

      int                  & rigidty()        { return m_rigidty;  }
      int const            & rigidty()  const { return m_rigidty;  }
      coupling_type        & coupling()       { return m_coupling; }
      coupling_type  const & coupling() const { return m_coupling; }

    protected:

      bool exist_stick(particle_type * A,particle_type * B)
      {
        typedef typename boost::indirect_iterator< typename stick_ptr_container::iterator, stick_type> stick_iterator;

        stick_ptr_container sticksA = m_stick_lut[A];
        stick_ptr_container sticksB = m_stick_lut[B];
        if(sticksA.empty())
          return false;
        if(sticksB.empty())
          return false;
        {
          stick_iterator  s   = stick_iterator( sticksA.begin() );
          stick_iterator  end = stick_iterator( sticksA.end()   );
          for(;s!=end;++s)
          {
            if( (s->A() == A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        {
          stick_iterator  s   = stick_iterator( sticksB.begin() );
          stick_iterator  end = stick_iterator( sticksB.end()   );
          for(;s!=end;++s)
          {
            if( (s->A()==A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        return false;
      }

      bool exist_spring(particle_type * A,particle_type * B)
      {
        typedef boost::indirect_iterator< typename spring_ptr_container::iterator, spring_type> spring_iterator;
        spring_ptr_container springsA = m_spring_lut[A];
        spring_ptr_container springsB = m_spring_lut[B];
        if(springsA.empty())
          return false;
        if(springsB.empty())
          return false;
        {
          spring_iterator  s   = spring_iterator( springsA.begin() );
          spring_iterator  end = spring_iterator( springsA.end()   );
          for(;s!=end;++s)
          {
            if( (s->A() == A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        {
          spring_iterator  s   = spring_iterator( springsB.begin() );
          spring_iterator  end = spring_iterator( springsB.end()   );
          for(;s!=end;++s)
          {
            if( (s->A()==A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        return false;
      }


      void add_stick(particle_type * A,particle_type * B)
      {
        if(A==B)
          return;

        m_sticks.push_back(stick_type());
          stick_type * s = &m_sticks.back();
          this->add_constraint( s );
          s->init(A,B);
          m_stick_lut[A].push_back(s);
          m_stick_lut[B].push_back(s);

      }

      void add_spring(particle_type * A,particle_type * B)
      {
        if(A==B)
          return;

        m_springs.push_back(spring_type());
          spring_type * s = &m_springs.back();
          this->add_force( s );
          s->init(A,B);
          m_spring_lut[A].push_back(s);
          m_spring_lut[B].push_back(s);

      }


      template<typename vertex_iterator_container>
      void traverse(
          vertex_iterator root
        , vertex_iterator from
        , bool create_sticks
        , bool create_springs
        , int dist
        , vertex_iterator_container & visited
      )
      {
        if( dist >= m_rigidty )
          return;

        visited.push_back(from);
        from->m_tag = 1;

        vertex_halfedge_circulator h(*from),hend;
        for(;h!=hend;++h)
        {
          vertex_iterator to = h->get_destination_iterator();
          if( to->m_tag == 0 )
          {
            particle_type * A = &(  m_coupling.particle( (*root) )   );
            particle_type * B = &(  m_coupling.particle(  (*to)  )   );

            if(create_sticks && !exist_stick(A,B))
              add_stick(A,B);

            if(create_springs && !exist_spring(A,B))
              add_spring(A,B);

            traverse( root, to, create_sticks, create_springs, dist+1, visited);
          }
        }
      }

    public:

      SurfaceMesh()
        : m_rigidty(2)
        , m_sticks()
        , m_stick_lut()
        , m_springs()
        , m_spring_lut()
      {}

      virtual ~SurfaceMesh() {}

    public:

      virtual void clear()
      {
        base_class::clear();

        m_coupling.clear();

        m_sticks.clear();
        m_springs.clear();
        m_stick_lut.clear();
        m_spring_lut.clear();
      }

      virtual void init(mesh_type /*const*/ & mesh, bool create_sticks, bool create_springs)
      {
        clear();

        m_coupling.init(*this,mesh);

        mesh::clear_vertex_tags(m_coupling.mesh());

        vertex_iterator end   = m_coupling.mesh().vertex_end();
        vertex_iterator v     = m_coupling.mesh().vertex_begin();
        for(;v!=end;++v)
        {
          std::list<vertex_iterator> visited;

          traverse( v, v, create_sticks, create_springs, 0, visited);

          for(typename std::list<vertex_iterator>::iterator w = visited.begin();w!=visited.end();++w)
            (*w)->m_tag = 0;
        }

        m_stick_lut.clear();
        m_spring_lut.clear();
      }

    };
  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_SURFACE_MESH_H
#endif
