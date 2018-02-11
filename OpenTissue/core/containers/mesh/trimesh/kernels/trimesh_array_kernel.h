#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_KERNELS_TRIMESH_ARRAY_KERNEL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_KERNELS_TRIMESH_ARRAY_KERNEL_H
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

#include <list>
#include <vector>
#include <cassert>

namespace OpenTissue
{
  namespace trimesh
  {

    /**
    *
    */
    template< typename vertex_type_, typename face_type_ >
    class TriMeshArrayKernel
    {
    public:

      typedef vertex_type_                               vertex_type;
      typedef face_type_                                 face_type;
      typedef TriMeshArrayKernel<vertex_type, face_type> kernel_type;

    public:

      typedef std::size_t                                          index_type;
      typedef typename std::vector<vertex_type>::size_type         size_type;
      typedef typename std::vector<vertex_type>::iterator          vertex_iterator;
      typedef typename std::vector<face_type>::iterator            face_iterator;
      typedef typename std::vector<vertex_type>::const_iterator    const_vertex_iterator;
      typedef typename std::vector<face_type>::const_iterator      const_face_iterator;

    private:

      /**
      * Default Constructed Handle:
      *
      * Null handle, Equivalent to a null-pointer or out-of-bound state.
      *
      */
      class Handle
      {
      protected:

        index_type m_idx;

      public:

        Handle()
          : m_idx(~0u)
        {}

        explicit Handle(index_type idx)
          : m_idx(idx)
        {}

        Handle(Handle const & h)
          : m_idx(h.m_idx)
        {}

        Handle & operator=(Handle const & h){ m_idx = h.m_idx; return *this; }
        bool operator<(Handle const & h)const { return m_idx < h.m_idx;}
        bool operator==(Handle const & h) const {return (h.m_idx==m_idx);}
        bool operator!=(Handle const & h) const {return !((*this)==h);}
        index_type get_idx(void) const {  return m_idx; }
        bool is_null() const { return  (m_idx == ~0u); }
      };

    public:

      class vertex_handle : public Handle
      {
      public:
        vertex_handle():Handle(){}
        vertex_handle(index_type idx):Handle(idx){}
        vertex_handle(vertex_handle const & v):Handle(v){}
      };

      class face_handle : public Handle
      {
      public:
        face_handle():Handle(){}
        face_handle(index_type idx):Handle(idx){}
        face_handle(face_handle const & f):Handle(f){}
      };

    public:

      static vertex_handle const & null_vertex_handle()
      {
        static vertex_handle h;
        return h;
      }
      static face_handle const & null_face_handle()
      {
        static face_handle h;
        return h;
      }

    private:

      std::vector<vertex_type>              m_vertices;
      std::vector<face_type>                m_faces;
      std::vector<bool>                     m_used_vertex;
      std::vector<bool>                     m_used_face;

    public:

      vertex_iterator         vertex_begin()       { return m_vertices.begin(); }
      vertex_iterator         vertex_end()         { return m_vertices.end(); }
      face_iterator           face_begin()         { return m_faces.begin(); }
      face_iterator           face_end()           { return m_faces.end(); }

      const_vertex_iterator   vertex_begin() const { return m_vertices.begin(); }
      const_vertex_iterator   vertex_end()   const { return m_vertices.end(); }
      const_face_iterator     face_begin()   const { return m_faces.begin(); }
      const_face_iterator     face_end()     const { return m_faces.end(); }

      size_type size_faces()    const { return m_faces.size(); }
      size_type size_vertices() const { return m_vertices.size(); }

    public:

      TriMeshArrayKernel()
      {}

      explicit TriMeshArrayKernel(TriMeshArrayKernel const & other_kernel)
      { *this = other_kernel; }

    public:

      TriMeshArrayKernel & operator=(TriMeshArrayKernel const & rhs)
      {
        m_vertices = rhs.m_vertices;
        m_faces = rhs.m_faces;
        m_used_vertex = rhs.m_used_vertex;
        m_used_face = rhs.m_used_face;
        return (*this);
      }

    protected:

      vertex_handle create_vertex()
      {
        m_vertices.push_back(vertex_type());
        vertex_iterator last = m_vertices.end();
        --last;
        index_type new_idx = m_used_vertex.size();
        m_used_vertex.push_back(true);
        vertex_handle h(new_idx);
        trimesh_core_access::set_self_handle( last,    h);
        return h;
      }

      face_handle create_face()
      {
        m_faces.push_back(face_type());
        face_iterator last = m_faces.end();
        --last;
        index_type new_idx = m_used_face.size();
        m_used_face.push_back(true);
        face_handle h(new_idx);
        trimesh_core_access::set_self_handle( last,    h);
        return h;
      }

      void erase_vertex(vertex_handle const & v)
      {
        if(is_valid_vertex_handle(v))
          m_used_vertex[v.get_idx()] = false;
      }

      void erase_face(face_handle const & f)
      {
        if(is_valid_face_handle(f))
          m_used_face[f.get_idx()] = false;
      }

    public:

      vertex_handle get_vertex_handle(index_type idx) const
      {
        assert(idx>=0);
        assert(idx<m_used_vertex.size());
        if(m_used_vertex[idx])
          return vertex_handle(idx);
        return null_vertex_handle();
      };

      face_handle get_face_handle(index_type idx) const
      {
        assert(idx>=0);
        assert(idx<m_used_face.size());
        if(m_used_face[idx])
          return face_handle(idx);
        return null_face_handle();
      }


      vertex_iterator get_vertex_iterator(vertex_handle const & v)
      {
        if( ! is_valid_vertex_handle(v) )
          return vertex_end();
        return  (m_vertices.begin() + v.get_idx());
      }

      face_iterator get_face_iterator(face_handle const & f)
      {
        if( ! is_valid_face_handle(f) )
          return face_end();
        return  m_faces.begin() + f.get_idx();
      };

      const_vertex_iterator get_vertex_iterator(vertex_handle const & v) const
      {
        if( ! is_valid_vertex_handle(v) )
          return vertex_end();
        return  (m_vertices.begin() + v.get_idx());
      }

      const_face_iterator get_face_iterator(face_handle const & f) const
      {
        if( ! is_valid_face_handle(f) )
          return face_end();
        return  m_faces.begin() + f.get_idx();
      }

      void clear()
      {
        m_vertices.clear();
        m_faces.clear();
        m_used_vertex.clear();
        m_used_face.clear();
      }

    public:

      bool is_valid_vertex_handle(vertex_handle const & v) const
      {
        if(v.is_null())
          return false;
        assert(v.get_idx()>=0);
        assert(v.get_idx()<m_used_vertex.size());
        return m_used_vertex[v.get_idx()];
      }

      bool is_valid_face_handle(face_handle const & f) const
      {
        if(f.is_null())
          return false;
        assert(f.get_idx()>=0);
        assert(f.get_idx()<m_used_face.size());
        return m_used_face[f.get_idx()];
      }

    public:

      void pack()
      {
        assert(false || !"Sorry not implemented yet");
      }

    };

  } // namespace trimesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_TRIMESH_KERNELS_TRIMESH_ARRAY_KERNEL2_H
#endif
