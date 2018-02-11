#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TRIANGLE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TRIANGLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_base_shape.h>
#include <OpenTissue/utility/utility_class_id.h>

#include <OpenTissue/core/geometry/geometry_barycentric.h>

#include <cassert>

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * A Triangle Class.
    * Observe that the vertices of the triangle can be bound
    * to vector3's allocated elsewhere, this is useful for binding
    * a triangle to a underlying deforming geomtry.
    */
    template< typename math_types_ >
    class Triangle
      : public BaseShape< math_types_ >
      , public OpenTissue::utility::ClassID< Triangle<math_types_> >
    {
    public:

      typedef          math_types_                   math_types;
      typedef typename math_types::real_type         real_type;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::matrix3x3_type    matrix3x3_type;

    protected:

      typedef typename math_types::value_traits      value_traits;

    protected:

      vector3_type * m_p0;    ///< Triangle vertex number 1
      vector3_type * m_p1;    ///< Triangle vertex number 2
      vector3_type * m_p2;    ///< Triangle vertex number 3
      bool           m_bound; ///< Boolan flag indicating whether triangle vertices are bound to other data (allocated elsewhere).

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Triangle<math_types_> >::class_id(); }

      Triangle()
        : m_p0(0)
        , m_p1(0)
        , m_p2(0)
        , m_bound(false)
      {}

      virtual ~Triangle()
      {
        if(!m_bound)
        {
          if(m_p0)
            delete m_p0;
          if(m_p1)
            delete m_p1;
          if(m_p2)
            delete m_p2;
        }
      }

      Triangle(Triangle const & triangle)
      {
        m_p0 = 0;
        m_p1 = 0;
        m_p2 = 0;
        m_bound = false;
        set(triangle);
      }

      explicit Triangle(vector3_type const & p0_,vector3_type const & p1_,vector3_type const & p2_)
      {
        m_p0 = 0;
        m_p1 = 0;
        m_p2 = 0;
        m_bound = false;
        set(p0_,p1_,p2_);
      }

      /**
      * Specialized Constructor.
      * This constuctor makes the triangle store pointers to its vertex coordinates.
      *
      * @param p0
      * @param p1
      * @param p2
      */
      explicit Triangle(vector3_type * p0_,vector3_type * p1_,vector3_type * p2_)
      {
        m_p0 = 0;
        m_p1 = 0;
        m_p2 = 0;
        m_bound = false;
        bind(p0_,p1_,p2_);
      }

      /**
      * Specialized Constructor.
      *
      * @param face
      */
      template<typename face_type>
        Triangle(face_type * f)
        : m_p0(0)
        , m_p1(0)
        , m_p2(0)
        , m_bound(false)
      {
        set(f);
      }

      /**
      * Assignment Operator.
      *
      * @param triangle
      */
      Triangle const & operator=(Triangle const & triangle)
      {
        set(triangle);
        return *this;
      }

    public:

      template<typename face_type>
      void set(face_type * f)
      {
        typedef typename face_type::mesh_type                mesh_type;
        typedef typename mesh_type::face_vertex_circulator   face_vertex_circulator;
        if( valency(*f)!=3)
          return;
        face_vertex_circulator v0(*f);
        face_vertex_circulator v1(*f);++v1;
        face_vertex_circulator v2(*f);++v2;++v2;
        bind( &(v0->m_coord), &(v1->m_coord), &(v2->m_coord) );
      }

      void set(Triangle const & triangle)
      {
        if(triangle.m_bound)
        {
          bind(triangle.m_p0,triangle.m_p1,triangle.m_p2);
        }
        else
        {
          unbind();
          set(*(triangle.m_p0),*(triangle.m_p1),*(triangle.m_p2));
        }
      }

      void set(const vector3_type & p0_, const vector3_type & p1_, const vector3_type & p2_)
      {
        unbind();
        assert(m_p0 || !"Triangle::set(): p0 member was null");
        assert(m_p1 || !"Triangle::set(): p1 member was null");
        assert(m_p2 || !"Triangle::set(): p2 member was null");
        *m_p0 = p0_;
        *m_p1 = p1_;
        *m_p2 = p2_;
      }

    public:

      /**
      * Bind
      *
      * @param p0
      * @param p1
      * @param p2
      */
      void bind(vector3_type * p0_, vector3_type * p1_, vector3_type * p2_)
      {
        assert(p0_  || !"Triangle::set(): p0 argument was null");
        assert(p1_  || !"Triangle::set(): p1 argument was null");
        assert(p2_  || !"Triangle::set(): p2 argument was null");
        if(m_bound)
        {
          m_p0 = p0_;
          m_p1 = p1_;
          m_p2 = p2_;
        }
        else
        {
          if(m_p0)
            delete m_p0;
          m_p0 = p0_;
          if(m_p1)
            delete m_p1;
          m_p1 = p1_;
          if(m_p2)
            delete m_p2;
          m_p2 = p2_;
        }
        m_bound = true;
      }

      void unbind()
      {
        if(!m_bound && m_p0 && m_p1 && m_p2)
          return;

        vector3_type * tmp = 0;

        tmp = m_p0;
        m_p0 = new vector3_type(0,0,0);
        if(tmp)
          (*m_p0) = (*tmp);

        tmp = m_p1;
        m_p1 = new vector3_type(0,0,0);
        if(tmp)
          (*m_p1) = (*tmp);

        tmp = m_p2;
        m_p2 = new vector3_type(0,0,0);
        if(tmp)
          (*m_p2) = (*tmp);

        m_bound = false;
      };

    public:
      /**
      * Compute Barycentric Coordinates.
      *
      * @param p    The point for which the barycentric coordinates should be computed.
      * @param w1    Upon return this parameter contains the value of the first barycentric coordinate.
      * @param w2    Upon return this parameter contains the value of the second barycentric coordinate.
      * @param w3    Upon return this parameter contains the value of the third barycentric coordinate.
      */
      void barycentric(vector3_type const & p,real_type & w1,real_type & w2,real_type & w3)const
      {
        OpenTissue::geometry::barycentric_algebraic((*m_p0),(*m_p1),(*m_p2),p,w1,w2,w3);
      }

    public:

      real_type area()const
      {
        assert(m_p0 || !"Triangle::area(): p0 member was null");
        assert(m_p1 || !"Triangle::area(): p1 member was null");
        assert(m_p2 || !"Triangle::area(): p2 member was null");
        vector3_type u = (*m_p1)-(*m_p0);
        vector3_type v = (*m_p2)-(*m_p0);
        vector3_type uXv = u % v;
        return 0.5*length(uXv);
      };

      /**
      * Retrieve Triangle Corners
      *
      * @param points
      */
      void compute_surface_points(std::vector<vector3_type> & points)const
      {
        assert(m_p0 || !"Triangle::compute_surface_points(): p0 member was null");
        assert(m_p1 || !"Triangle::compute_surface_points(): p1 member was null");
        assert(m_p2 || !"Triangle::compute_surface_points(): p2 member was null");
        points.push_back(*m_p0);
        points.push_back(*m_p1);
        points.push_back(*m_p2);
      }

      vector3_type get_support_point(vector3_type const & v) const
      {
        return v; // TODO!
      }

      vector3_type       & p0()      {  assert(m_p0); return (*m_p0); }
      vector3_type       & p1()      {  assert(m_p1); return (*m_p1); }
      vector3_type       & p2()      {  assert(m_p2); return (*m_p2); }
      vector3_type const & p0()const {  assert(m_p0); return (*m_p0); }
      vector3_type const & p1()const {  assert(m_p1); return (*m_p1); }
      vector3_type const & p2()const {  assert(m_p2); return (*m_p2); }

      vector3_type center() const
      {
        assert(m_p0 || !"Triangle::center(): p0 member was null");
        assert(m_p1 || !"Triangle::center(): p1 member was null");
        assert(m_p2 || !"Triangle::center(): p2 member was null");
        vector3_type tmp = ((*m_p0) +(*m_p1) +(*m_p2) )/3.;
        return tmp;
      }

      vector3_type normal()const
      {
        assert(m_p0 || !"Triangle::normal(): p0 member was null");
        assert(m_p1 || !"Triangle::normal(): p1 member was null");
        assert(m_p2 || !"Triangle::normal(): p2 member was null");
        vector3_type u = (*m_p1)-(*m_p0);
        vector3_type v = (*m_p2)-(*m_p0);
        return  unit(u%v);
      }


      void translate(vector3_type const & T)
      {
        assert(m_p0 || !"Triangle::scale() p0 member was NULL");
        assert(m_p1 || !"Triangle::scale() p1 member was NULL");
        assert(m_p2 || !"Triangle::scale() p2 member was NULL");
        *m_p0 += T;
        *m_p1 += T;
        *m_p2 += T;
      }

      void rotate(matrix3x3_type const & R)
      {
        // TODO: rotate around center, I guess?!
      }

      /**
       * Uniform Scaling of Triangle.
       *
       * @param s    The scaling of the triangle.
       */
      void scale(real_type const & s)
      {
        assert(m_p0 || !"Triangle::scale() p0 member was NULL");
        assert(m_p1 || !"Triangle::scale() p1 member was NULL");
        assert(m_p2 || !"Triangle::scale() p2 member was NULL");
        vector3_type c = center();
        *m_p0 = (*m_p0 - c)*s + c;
        *m_p1 = (*m_p1 - c)*s + c;
        *m_p2 = (*m_p2 - c)*s + c;
      }

      /**
      * Compute Bounding Box.
      * This method computes an axis aligned bounding
      * box (AABB) that encloses the geometry.
      *
      * @param r           The position of the model frame (i.e the coordinate frame the geometry lives in).
      * @param R           The orientation of the model frame (i.e the coordinate frame the geometry lives in).
      * @param min_coord   Upon return holds the minimum corner of the box.
      * @param max_coord   Upon return holds the maximum corner of the box.
      *
      */
      void compute_collision_aabb(
        vector3_type const & r
        , matrix3x3_type const & R
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {
        assert(m_p0 || !"Triangle::compute_collision_aabb() p0 member was NULL");
        assert(m_p1 || !"Triangle::compute_collision_aabb() p1 member was NULL");
        assert(m_p2 || !"Triangle::compute_collision_aabb() p2 member was NULL");

        using std::min;
        using std::max;

        min_coord = min( *m_p0, min( *m_p1, *m_p2 ) );
        max_coord = max( *m_p0, max( *m_p1, *m_p2 ) );

      }

    };

  }  // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TRIANGLE_H
#endif
