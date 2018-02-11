#ifndef OPENTISSUE_DYNAMICS_SPH_COLLISION_POLICIES_SPH_TETRAHEDRA_POINTS_H
#define OPENTISSUE_DYNAMICS_SPH_COLLISION_POLICIES_SPH_TETRAHEDRA_POINTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/collision/sph_collision_type.h>
#include <OpenTissue/collision/spatial_hashing/spatial_hashing.h>
#include <OpenTissue/core/geometry/geometry_triangle.h>
#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/math/math_basic_types.h>

#include <vector>


namespace OpenTissue
{
  namespace sph
  {

    template <
      typename real_type_
      , typename vector3_type_
      , typename tetrahedron_type_
      , typename point_data_type
    >
    class TetrahedraPointsCollisionDetectionPolicy
    {
    public:
      typedef math::BasicMathTypes<real_type_, int>   math_types;
      typedef real_type_  real_type;
      typedef vector3_type_  vector3_type;
      typedef tetrahedron_type_  tetrahedron_type;
      typedef point_data_type  point_data;
      typedef CollisionType<real_type, vector3_type>  collision_type;
      typedef std::vector<collision_type>  collision_container;

    public:

      /**
      * template class TetrahedronWrapper
      *
      * Requires the  node_type  of  TetrahedronType_  to define
      *   point_type
      * and to have an accessible member function
      *   const point_type& vertex()
      *
      */
      class TetrahedronWrapper
      {
      public:
        typedef tetrahedron_type  tetrahedron_type_inner;
        typedef typename tetrahedron_type_inner::node_type  node_type;
        typedef typename node_type::point_type  point_type;

      public:
        TetrahedronWrapper(const tetrahedron_type_inner* tetrahedron)
          : m_tetrahedron(tetrahedron)
          //        , m_tag(false)
        {}

      public:

        const tetrahedron_type_inner* data() const
        {
          return m_tetrahedron;
        }
        /*
        bool& tagged() {
        return m_tag;
        }

        const bool& tagged() const {
        return m_tag;
        }
        */
        point_type min() const
        {
          using std::min;

          const point_type& p0 = m_tetrahedron->i()->vertex();
          const point_type& p1 = m_tetrahedron->j()->vertex();
          const point_type& p2 = m_tetrahedron->k()->vertex();
          const point_type& p3 = m_tetrahedron->m()->vertex();
          return min(p0, min(p1, min(p2, p3)));
        }

        point_type max() const
        {
          using std::max;

          const point_type& p0 = m_tetrahedron->i()->vertex();
          const point_type& p1 = m_tetrahedron->j()->vertex();
          const point_type& p2 = m_tetrahedron->k()->vertex();
          const point_type& p3 = m_tetrahedron->m()->vertex();
          return max(p0, max(p1, max(p2, p3)));
        }

      public:
        const tetrahedron_type_inner*  m_tetrahedron;
        //bool  m_tag;

      };


      /**
      * template class PointWrapper
      *
      * Requires the PointData_ to define
      *   point_type
      * and to have an accessible member function
      *   const point_type& position()
      *
      */
      class PointWrapper
      {
      public:
        typedef point_data  point_data_inner;
        typedef typename point_data_inner::point_type  point_type;

      public:
        PointWrapper(const point_data_inner* point)
          : m_point(point)
        {}

      public:
        const point_data_inner* data() const
        {
          return m_point;
        }

        const point_type& position() const
        {
          return m_point->position();
        }

        const point_type& min() const
        {
          return m_point->position();
        }

        const point_type& max() const
        {
          return m_point->position();
        }

      private:
        const point_data_inner*  m_point;

      };

    public:
      typedef TetrahedronWrapper  tetrahedron_wrapper;
      typedef typename tetrahedron_wrapper::tetrahedron_type_inner  tetrahedron;
      typedef PointWrapper  point_wrapper;
      typedef std::vector<tetrahedron_wrapper>  tetrahedron_container;
      typedef typename tetrahedron_container::iterator  tetrahedron_iterator;
      typedef geometry::Triangle<math_types>  Triangle;
      typedef geometry::Plane<math_types>  Plane;

    public:

      class CollisionPolicy
      {
      public:
        typedef typename math_types::real_type  real_type;
        typedef typename math_types::vector3_type  vector_inner;
        typedef typename tetrahedron_wrapper::point_type  t_point_type;
        typedef typename point_wrapper::point_type  p_point_type;
        typedef tetrahedron_wrapper  data_type;
        typedef point_wrapper  query_type;

        typedef OpenTissue::spatial_hashing::PrimeNumberHashFunction  hash_function;
        typedef OpenTissue::spatial_hashing::Grid<vector_inner, OpenTissue::math::Vector3<long>, data_type, hash_function>  hash_grid;

      public:

        struct ResultType
        {
          const point_data*  point;
          const tetrahedron_type*  tetrahedron;
          real_type  w0, w1, w2, w3;
        };

        typedef ResultType  result_type;
        typedef std::vector<result_type>  result_container;

      public:

        point_data min_coord(data_type const & d)const{return d.min();};
        point_data max_coord(data_type const & d)const{return d.max();};
        point_data min_coord(query_type const & q)const{return q.min();};
        point_data max_coord(query_type const & q)const{return q.max();};

        void reset(result_container& results)
        {
          results.clear();
        }

        void report(data_type& data, const query_type& query, result_container& results)
        {
          //--- First we do a quick rejection test. If the vertex is allready reported then simply ignore it!!!
          //        if (data.tagged())
          //          return;

          const tetrahedron_type* t = data.data();
          const t_point_type& pi = t->i()->vertex();
          const t_point_type& pj = t->j()->vertex();
          const t_point_type& pk = t->k()->vertex();
          const t_point_type& pm = t->m()->vertex();
          const p_point_type& p  = query.position();

          result_type result;

          OpenTissue::geometry::barycentric_algebraic(pi, pj, pk, pm, p, result.w0, result.w1, result.w2, result.w3);

          // NOTE: henrikd 2005-08-10 - epsilon is unused, so I commented it out
          //const real_type epsilon = 1./4096;

          //const real_type epsilon = 1./8192;
          //const real_type epsilon = 1./16384;
          if (result.w0 >= 0 &&
            result.w1 >= 0 &&
            result.w2 >= 0 &&
            result.w3 >= 0)
          {
            //          data.tagged() = true;
            result.point = query.data();
            result.tetrahedron = data.data();
            results.push_back(result);
          }
        }

      };

    public:
      typedef CollisionPolicy  collision_policy;
      typedef typename collision_policy::t_point_type  t_point_type;
      typedef typename collision_policy::p_point_type  p_point_type;
      typedef typename collision_policy::result_type  result_type;
      typedef typename collision_policy::result_container  result_container;
      //typedef OpenTissue::spatial_hashing::PointDataQuery<typename collision_policy::hash_grid, collision_policy>  collision_detection;
      typedef OpenTissue::spatial_hashing::AABBDataQuery<typename collision_policy::hash_grid, collision_policy>  collision_detection;

    public:

      void clear()
      {
        m_tetrahedra.clear();
      }

      template< typename TetrahedronIterator >
      void addObstacle(const TetrahedronIterator& begin, const TetrahedronIterator& end)
      {
        for (TetrahedronIterator t = begin; t != end; ++t)
          m_tetrahedra.push_back(tetrahedron_wrapper(&*t));

        refresh();
      }

      void refresh()
      {
        m_collision.init(m_tetrahedra.begin(), m_tetrahedra.end());
        m_collision(m_tetrahedra.begin(), m_tetrahedra.end());
      }

      bool collision(collision_type& collision, const point_data& query)
      {
        point_wrapper q(&query);
        result_container collisions;
        /*
        const tetrahedron_iterator end = m_tetrahedra.end();
        for (tetrahedron_iterator t = m_tetrahedra.begin(); t != end; ++t)
        t->tagged() = false;
        */
        m_collision(q, collisions, typename collision_detection::all_tag());
        if (collisions.empty())
          return false;

        static int miss = 0;
        typename result_container::const_iterator end = collisions.end();
        for (typename result_container::const_iterator c = collisions.begin(); c != end; ++c) {
          const result_type& coli = *c;

          const p_point_type& p0 = coli.point->position_old();
          const p_point_type& p1 = coli.point->position();
          const p_point_type dir = p0-p1;  // reversed direction
          //const p_point_type dir = -coli.point->velocity();

          const t_point_type& i = coli.tetrahedron->i()->vertex();
          const t_point_type& j = coli.tetrahedron->j()->vertex();
          const t_point_type& k = coli.tetrahedron->k()->vertex();
          const t_point_type& m = coli.tetrahedron->m()->vertex();

          // DEBUG
          dcoli.t4 = c->tetrahedron;
          dcoli.p0 = p0; dcoli.p1 = p1;
          dcoli.i = i; dcoli.j = j; dcoli.k = k; dcoli.m = m;
          // DEBUG

          // up to 4 triangle-line tests
          /*
          if (intersectTriangleLine(collision, p0, p1, Triangle(j, k, m)))
          return true;
          if (intersectTriangleLine(collision, p0, p1, Triangle(i, j, m)))
          return true;
          if (intersectTriangleLine(collision, p0, p1, Triangle(k, i, m)))
          return true;
          if (intersectTriangleLine(collision, p0, p1, Triangle(i, k, j)))
          return true;
          */
          if (intersectTriangleLine(collision, p0+dir, p1, Triangle(j, k, m)))
            return true;
          if (intersectTriangleLine(collision, p0+dir, p1, Triangle(i, j, m)))
            return true;
          if (intersectTriangleLine(collision, p0+dir, p1, Triangle(k, i, m)))
            return true;
          if (intersectTriangleLine(collision, p0+dir, p1, Triangle(i, k, j)))
            return true;

        }

        // something is really really fucked up :(
        std::cout << "MISS: " << ++miss << std::endl;
        collision.contact() = 0;
        return true;
      }

      bool intersectTriangleLine(collision_type& collision, const p_point_type& p0, const p_point_type& p1, const Triangle& t) const
      {
        Plane plane(t.p0(), t.p1(), t.p2());
        real_type dist_p0 = plane.signed_distance(p0);
        real_type dist_p1 = plane.signed_distance(p1);
        if (dist_p0 >= 0. && dist_p1 >= 0.)
          return false;
        if (dist_p0 < 0. && dist_p1 < 0.)
          return false;
        double const s = dist_p0/(dist_p0-dist_p1);
        p_point_type const u = p0 + s*(p1-p0);
        real_type w1, w2, w3;
        t.barycentric(u, w1, w2, w3);
        if (w1 < 0. || w2 < 0. || w3 < 0.)
          return false;

        collision.contact() = u;
        collision.normal() = t.normal();  // already normalized
        collision.penetration() = length(u-p1);

        return true;
      }

    public:
      struct ColiDebug {
        ColiDebug():t4(NULL){}
        const tetrahedron_type* t4;
        p_point_type p0, p1;
        t_point_type i, j, k, m;
      } dcoli;

    protected:
      tetrahedron_container  m_tetrahedra;
      collision_detection  m_collision;

    };

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_COLLISION_POLICIES_SPH_TETRAHEDRA_POINTS_H
#endif
