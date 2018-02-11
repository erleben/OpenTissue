#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_MESH_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/trimesh/trimesh_vertex.h>
#include <OpenTissue/core/containers/mesh/trimesh/trimesh_face.h>
#include <OpenTissue/core/containers/mesh/trimesh/trimesh_core_access.h>

#include <algorithm>

namespace OpenTissue
{
  namespace trimesh
  {
    namespace detail
    {

      /** 
      * The first template argument is supposed to be a math types type binder.
      * OpenTissue provides a simple basic math type-binder in the math sub-library.
      * See OpenTissue::math::BasicMathTypes<real_type, size_type>     
      *
      * The next two template arguments are supposed to be vertex traits,
      * and face traits. The last template argument is the trimesh kernel
      * type that is supposed to be used.
      */
      template<
        typename M
        , typename V
        , typename F
        , template <typename, typename> class K
      >
      class TMesh 
        : public K<
          TriMeshVertex< TMesh< M, V, F, K> >
        , TriMeshFace< TMesh< M, V, F, K> >
        >
      {
      public:

        typedef          M                                     math_types;
        typedef          V                                     vertex_traits;
        typedef          F                                     face_traits;
        typedef          TMesh< M, V, F, K  >                  mesh_type;
        typedef          TriMeshVertex<mesh_type>              vertex_type;
        typedef          TriMeshFace<mesh_type>                face_type;
        typedef          K< vertex_type, face_type  >          kernel_type;
        typedef typename kernel_type::vertex_handle            vertex_handle;
        typedef typename kernel_type::face_handle              face_handle;
        typedef typename kernel_type::vertex_iterator          vertex_iterator;
        typedef typename kernel_type::face_iterator            face_iterator;
        typedef typename kernel_type::const_vertex_iterator    const_vertex_iterator;
        typedef typename kernel_type::const_face_iterator      const_face_iterator;

      protected:

        /**
        * Face Vertex Circulator.
        *
        * Example Usage:
        *
        *  typedef TriMeshFaceVertexCirculator<vertex_type> face_vertex_circualtor;
        *  typedef TriMeshFaceVertexCirculator<vertex_type const> const_face_vertex_circualtor;
        *
        *  face_vertex_circualtor circ( &(*f_iter) );
        *  face_vertex_circualtor end(  );
        *  std::for_each(circ,end, ... );
        *
        * Similar for the const iterator.
        */
        template<typename value_type>
        class  TriMeshFaceVertexCirculator
        {
        private:

          unsigned int m_idx;    ///< Local vertex index of face.
          face_type *  m_face;   ///< The face of the circulator.
          bool         m_active; ///< Boolean indicater.

        public:
          TriMeshFaceVertexCirculator()
            : m_idx(0)
            , m_face(0)
            , m_active(false)
          {}

          explicit TriMeshFaceVertexCirculator( face_type const & f)
            : m_idx(0)
            , m_face(const_cast<face_type*>(&f))
            , m_active(false)
          {}

          template <class OtherValue>
          TriMeshFaceVertexCirculator(
            TriMeshFaceVertexCirculator<OtherValue> const& other
            )
            : m_idx(other.m_idx)
            , m_face(other.m_face)
            , m_active(other.m_active)
          {}

        public:

          template <class OtherValue>
          bool operator==(TriMeshFaceVertexCirculator<OtherValue> const& /*other*/) const
          {
            return (m_active && m_idx == 0);
          }

          template <class OtherValue>
          bool operator!=(TriMeshFaceVertexCirculator<OtherValue> const& other) const
          {
            return !( *this == other);
          }

          TriMeshFaceVertexCirculator & operator++()
          {
            m_active = true;
            m_idx = (m_idx + 1)%3;
            return *this;
          }

          TriMeshFaceVertexCirculator & operator--()
          {
            m_active = true;
            m_idx = (m_idx - 1)%3;
            return *this;
          }

        public:

          value_type & operator*() const
          {
            if(m_idx==0)
              return *(m_face->get_vertex0_iterator());
            if(m_idx==1)
              return *(m_face->get_vertex1_iterator());
            //if(m_idx==2)
            return *(m_face->get_vertex2_iterator());
          }

          value_type * operator->() const
          {
            if(m_idx==0)
              return &(*(m_face->get_vertex0_iterator()));
            if(m_idx==1)
              return &(*(m_face->get_vertex1_iterator()));
            //if(m_idx==2)
            return &(*(m_face->get_vertex2_iterator()));
          }

        };

      public:

        typedef TriMeshFaceVertexCirculator<vertex_type>        face_vertex_circulator;
        typedef TriMeshFaceVertexCirculator<vertex_type const>  const_face_vertex_circulator;

      private:

        struct assign_owner
        {
          assign_owner(mesh_type * new_owner)
            : m_new_owner(new_owner)
          {};

          template <typename feature_type>
          void operator() (feature_type & f)
          {
            trimesh_core_access::set_owner( (&f), m_new_owner);
          }

          mesh_type * m_new_owner;
        };

      public:

        TMesh(){}

        ~TMesh() { this->clear(); }

        explicit TMesh(TMesh const & m)  { (*this) = m; }

        TMesh & operator=(TMesh const & mesh)
        {
          kernel_type::operator=(mesh);

          //--- Reassign owner pointers of copied data
          std::for_each( this->vertex_begin(),   this->vertex_end(),   assign_owner(this) );
          std::for_each( this->face_begin(),     this->face_end(),     assign_owner(this) );

          return *this;
        }

      public:

        vertex_handle add_vertex()
        {
          vertex_handle v = this->create_vertex();
          vertex_iterator vit = get_vertex_iterator(v);
          trimesh_core_access::set_owner(vit,this);
          return v;
        };

        template<typename vector3_type>
        vertex_handle add_vertex(vector3_type const & coord)
        {
          vertex_handle v = add_vertex();
          get_vertex_iterator(v)->m_coord = coord;
          return v;
        };

        face_handle add_face(vertex_handle const & v0,vertex_handle const & v1,vertex_handle const & v2)
        {
          if(! is_valid_vertex_handle(v0) )
            return this->null_face_handle();
          if(! is_valid_vertex_handle(v1) )
            return this->null_face_handle();
          if(! is_valid_vertex_handle(v2) )
            return this->null_face_handle();
          if(v0==v1)
            return this->null_face_handle();
          if(v0==v2)
            return this->null_face_handle();
          if(v1==v2)
            return this->null_face_handle();

          //--- Create the face
          face_handle f = this->create_face();
          face_iterator fit = get_face_iterator(f);
          trimesh_core_access::set_owner(fit,this);

          trimesh_core_access::set_vertex0_handle(fit,v0);
          trimesh_core_access::set_vertex1_handle(fit,v1);
          trimesh_core_access::set_vertex2_handle(fit,v2);

          trimesh_core_access::increment_face_counter(fit->get_vertex0_iterator());
          trimesh_core_access::increment_face_counter(fit->get_vertex1_iterator());
          trimesh_core_access::increment_face_counter(fit->get_vertex2_iterator());

          return f;
        };

        template<typename vertex_handle_iterator>
        face_handle add_face(vertex_handle_iterator begin,vertex_handle_iterator end)
        {
          face_handle face;
          vertex_handle_iterator v0 = begin;
          vertex_handle_iterator v1 = v0; ++v1;
          vertex_handle_iterator v2 = v1; ++v2;
          //--- there may be more than three vertices, so we simply use
          //--- an ear-clipping algorithm to generate triangular faces
          //---
          //--- This of course only works on meshes with convex faces!!!!
          while(v2!=end)
          {
            face = add_face( *v0, *v1, *v2 );
            v1 = v2;
            ++v2;
          }
          //--- Since multiple faces may have been created, we simply choose
          //--- to return a handle to the last created face.
          return face;
        }

        bool remove_vertex(vertex_handle const & v)
        {
          if(!is_valid_vertex_handle(v))
          {
            assert(!"TMesh::remove_vertex(...): Invalid vertex handle");
            return false;
          }
          return remove_vertex( get_vertex_iterator(v) );
        };

        bool remove_vertex(vertex_iterator v)
        {
          if(v->get_face_count()>0)
          {
            assert(!"TMesh::remove_vertex(...): Could not remove vertex because it is used by a face");
            return false;
          }
          erase_vertex(v->get_handle());
          return true;
        };

        bool remove_face(face_handle const & f)
        {
          if(!is_valid_face_handle(f))
            return false;
          return remove_face( get_face_iterator(f) );
        };

        bool remove_face(face_iterator f)
        {
          //--- Make sure face is not ``pointing'' to something
          trimesh_core_access::decrement_face_counter(f->get_vertex0_iterator());
          trimesh_core_access::decrement_face_counter(f->get_vertex1_iterator());
          trimesh_core_access::decrement_face_counter(f->get_vertex2_iterator());
          //--- Ask kernel to remove face
          erase_face(f->get_handle());
          return true;
        };

      };

    } // namespace detail
  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_TRIMESH_MESH_H
#endif
