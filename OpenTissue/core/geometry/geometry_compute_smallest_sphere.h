#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SMALLEST_SPHERE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SMALLEST_SPHERE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>
#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/geometry/geometry_circumscribed_sphere.h>
#include <OpenTissue/collision/intersect/intersect_point_sphere.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace geometry
  {
    namespace detail
    {

      /**
      * Compute Smallest Enclosing Sphere.
      */
      class ComputeSmallestSphere
      {
      private:

        /**
        *
        * Indices of points that support current minimum volume sphere
        */
        template<typename vector3_type_>
        class support_type
        {
        public:

          typedef vector3_type_                      vector3_type;
          typedef typename vector3_type::value_type  real_type;

        public:

          int m_cnt;                 ///< Number of points in support
          vector3_type m_points[4];  ///< The coordinates of points in the support

        public:

          bool contains (vector3_type const & point)
          {
            real_type epsilon = math::working_precision<real_type>(10);

            for (int i = 0; i < m_cnt; ++i)
            {
              vector3_type diff = point - m_points[i];
              if ( diff*diff < epsilon )
                return true;
            }
            return false;
          }
        };

        /**
        * Point is contained in sphere test.
        *
        * @param p                   A point that should be tested for inclusion in a sphere.
        * @param sphere              The sphere that p should be tested against.
        * @param squared distance    Upon return this argument holds the squared distance of the point to the sphere surface.
        */
        template<typename vector3_type, typename sphere_type, typename real_type>
        bool contains(vector3_type const & p, sphere_type const & sphere, real_type & squared_distance)
        {
          real_type test = sqr_length(p - sphere.center());
          squared_distance = test - sphere.squared_radius();
          return squared_distance <= 0.0;
        }

        template<typename vector3_type, typename support_type, typename sphere_type>
        void update_support1 ( vector3_type const & point, support_type & support, sphere_type & minimal)
        {
          vector3_type const & p0 = support.m_points[0];
          vector3_type const & p1 = point;
          compute_circumscribed_sphere(p0,p1,minimal);
          support.m_cnt = 2;
          support.m_points[1] = p1;
        }

        template<typename vector3_type, typename support_type, typename sphere_type>
        void update_support2 ( vector3_type const & point, support_type & support, sphere_type & minimal)
        {
          typedef typename vector3_type::value_type  real_type;

          static sphere_type spheres[2];
          vector3_type const & p0 = support.m_points[0];
          vector3_type const & p1 = support.m_points[1];
          vector3_type const & p2 = point;
          real_type min_radius = math::detail::highest<real_type>();
          int index = -1;
          compute_circumscribed_sphere(p0,p2,spheres[0]);
          compute_circumscribed_sphere(p1,p2,spheres[1]);
          if ( OpenTissue::intersect::point_sphere(p1, spheres[0] ) )
          {
            min_radius = spheres[0].radius();
            index = 0;
          }
          if ( spheres[1].radius() < min_radius )
          {
            if ( OpenTissue::intersect::point_sphere(p0, spheres[1]) )
            {
              min_radius = spheres[1].radius();
              index = 1;
            }
          }
          if ( index != -1 )
          {
            minimal = spheres[index];
            support.m_points[1-index] = p2;
          }
          else
          {
            compute_circumscribed_sphere(p0,p1,p2, minimal);
            assert(minimal.radius() < min_radius);
            support.m_cnt = 3;
            support.m_points[2] = p2;
          }
        }

        template<typename vector3_type, typename support_type, typename sphere_type>
        void update_support3 ( vector3_type const & point, support_type & support, sphere_type & minimal)
        {
          typedef typename vector3_type::value_type  real_type;

          vector3_type const & p0 = support.m_points[0];
          vector3_type const & p1 = support.m_points[1];
          vector3_type const & p2 = support.m_points[2];
          vector3_type const & p3 = point;

          sphere_type spheres[6];
          real_type min_radius = math::detail::highest<real_type>();
          int index = -1;

          compute_circumscribed_sphere(p0,p3,spheres[0]);
          compute_circumscribed_sphere(p1,p3,spheres[1]);
          compute_circumscribed_sphere(p2,p3,spheres[2]);
          compute_circumscribed_sphere(p0,p1,p3,spheres[3]);
          compute_circumscribed_sphere(p0,p2,p3,spheres[4]);
          compute_circumscribed_sphere(p1,p2,p3,spheres[5]);

          if ( OpenTissue::intersect::point_sphere(p1, spheres[0]) && OpenTissue::intersect::point_sphere( p2, spheres[0] ) )
          {
            min_radius = spheres[0].radius();
            index = 0;
          }
          if ( spheres[1].radius() < min_radius && OpenTissue::intersect::point_sphere(p0,spheres[1]) && OpenTissue::intersect::point_sphere(p2,spheres[1]) )
          {
            min_radius = spheres[1].radius();
            index = 1;
          }
          if ( spheres[2].radius() < min_radius && OpenTissue::intersect::point_sphere(p0, spheres[2]) && OpenTissue::intersect::point_sphere(p1,spheres[2]) )
          {
            min_radius = spheres[2].radius();
            index = 2;
          }
          if ( spheres[3].radius() < min_radius && OpenTissue::intersect::point_sphere(p2, spheres[3]) )
          {
            min_radius = spheres[3].radius();
            index = 3;
          }
          if ( spheres[4].radius() < min_radius && OpenTissue::intersect::point_sphere(p1,spheres[4]) )
          {
            min_radius = spheres[4].radius();
            index = 4;
          }
          if ( spheres[5].radius() < min_radius && OpenTissue::intersect::point_sphere(p0, spheres[5]) )
          {
            min_radius = spheres[5].radius();
            index = 5;
          }

          switch ( index )
          {
          case 0:
            minimal = spheres[0];
            support.m_cnt = 2;
            support.m_points[1] = p3;
            break;
          case 1:
            minimal = spheres[1];
            support.m_cnt = 2;
            support.m_points[0] = p3;
            break;
          case 2:
            minimal = spheres[2];
            support.m_cnt = 2;
            support.m_points[0] = support.m_points[2];
            support.m_points[1] = p3;
            break;
          case 3:
            minimal = spheres[3];
            support.m_points[2] = p3;
            break;
          case 4:
            minimal = spheres[4];
            support.m_points[1] = p3;
            break;
          case 5:
            minimal = spheres[5];
            support.m_points[0] = p3;
            break;
          default:
            compute_circumscribed_sphere(p0,p1,p2,p3,minimal);
            assert(minimal.radius() < min_radius);
            support.m_cnt = 4;
            support.m_points[3] = p3;
            break;
          }
        }

        template<typename vector3_type, typename support_type, typename sphere_type>
        void update_support4 ( vector3_type const & point, support_type & support, sphere_type & minimal)
        {
          typedef typename vector3_type::value_type  real_type;

          vector3_type const * p[4] = { &support.m_points[0], &support.m_points[1], &support.m_points[2], &support.m_points[3]  };
          vector3_type const & p4 = point;

          // permutations of type 1
          int pi1[4][4] =
          {
            {0, /*4*/ 1,2,3},
            {1, /*4*/ 0,2,3},
            {2, /*4*/ 0,1,3},
            {3, /*4*/ 0,1,2}
          };

          // permutations of type 2
          int pi2[6][4] =
          {
            {0,1, /*4*/ 2,3},
            {0,2, /*4*/ 1,3},
            {0,3, /*4*/ 1,2},
            {1,2, /*4*/ 0,3},
            {1,3, /*4*/ 0,2},
            {2,3, /*4*/ 0,1}
          };

          // permutations of type 3
          int pi3[4][4] =
          {
            {0,1,2, /*4*/ 3},
            {0,1,3, /*4*/ 2},
            {0,2,3, /*4*/ 1},
            {1,2,3, /*4*/ 0}
          };

          sphere_type spheres[14];
          real_type min_radius = math::detail::highest<real_type>();
          int index = -1;
          real_type min_distance = math::detail::highest<real_type>();
          real_type squared_distance;
          int iMinIndex = -1;
          int k = 0;  // sphere index

          // permutations of type 1
          int j;
          for (j = 0; j < 4; ++j, ++k)
          {
            compute_circumscribed_sphere(*p[pi1[j][0]],p4,spheres[k]);

            if ( spheres[k].radius() < min_radius )
            {
              if ( contains(*p[pi1[j][1]],spheres[k],squared_distance) && contains(*p[pi1[j][2]],spheres[k],squared_distance) && contains(*p[pi1[j][3]],spheres[k],squared_distance))
              {
                min_radius = spheres[k].radius();
                index = k;
              }
              else if ( squared_distance < min_distance )
              {
                min_distance = squared_distance;
                iMinIndex = k;
              }
            }
          }

          // permutations of type 2
          for (j = 0; j < 6; ++j, ++k)
          {
            compute_circumscribed_sphere(*p[pi2[j][0]],*p[pi2[j][1]],p4,spheres[k]);
            if ( spheres[k].radius() < min_radius )
            {
              if ( contains( *p[pi2[j][2]],spheres[k],squared_distance) && contains(*p[pi2[j][3]],spheres[k],squared_distance) )
              {
                min_radius = spheres[k].radius();
                index = k;
              }
              else if ( squared_distance < min_distance )
              {
                min_distance = squared_distance;
                iMinIndex = k;
              }
            }
          }

          // permutations of type 3
          for (j = 0; j < 4; ++j, ++k)
          {
            compute_circumscribed_sphere(*p[pi3[j][0]],*p[pi3[j][1]],*p[pi3[j][2]],p4,spheres[k]);
            if ( spheres[k].radius() < min_radius )
            {
              if ( contains(*p[pi3[j][3]],spheres[k],squared_distance) )
              {
                min_radius = spheres[k].radius();
                index = k;
              }
              else if ( squared_distance < min_distance )
              {
                min_distance = squared_distance;
                iMinIndex = k;
              }
            }
          }

          // Theoretically, index >= 0 should happen, but floating point round-off
          // error can lead to this.  When this happens, the sphere is chosen that
          // has the minimum absolute errors between points (barely) outside the
          // sphere and the sphere.
          if ( index == -1 )
            index = iMinIndex;

          minimal = spheres[index];

          switch ( index )
          {
          case 0:
            support.m_cnt = 2;
            support.m_points[1] = p4;
            break;
          case 1:
            support.m_cnt = 2;
            support.m_points[0] = p4;
            break;
          case 2:
            support.m_cnt = 2;
            support.m_points[0] = support.m_points[2];
            support.m_points[1] = p4;
            break;
          case 3:
            support.m_cnt = 2;
            support.m_points[0] = support.m_points[3];
            support.m_points[1] = p4;
            break;
          case 4:
            support.m_cnt = 3;
            support.m_points[2] = p4;
            break;
          case 5:
            support.m_cnt = 3;
            support.m_points[1] = p4;
            break;
          case 6:
            support.m_cnt = 3;
            support.m_points[1] = support.m_points[3];
            support.m_points[2] = p4;
            break;
          case 7:
            support.m_cnt = 3;
            support.m_points[0] = p4;
            break;
          case 8:
            support.m_cnt = 3;
            support.m_points[0] = support.m_points[3];
            support.m_points[2] = p4;
            break;
          case 9:
            support.m_cnt = 3;
            support.m_points[0] = support.m_points[3];
            support.m_points[1] = p4;
            break;
          case 10:
            support.m_points[3] = p4;
            break;
          case 11:
            support.m_points[2] = p4;
            break;
          case 12:
            support.m_points[1] = p4;
            break;
          case 13:
            support.m_points[0] = p4;
            break;
          }
        }

      public:

        template<typename vector3_iterator, typename sphere_type>
        void operator()(vector3_iterator begin, vector3_iterator end, sphere_type & minimal)
        {
          assert(begin!=end || !"ComputeSmallestSphere::operator(): begin was equal to end");

          typedef typename sphere_type::real_type     real_type;
          typedef typename sphere_type::vector3_type  vector3_type;
          support_type<vector3_type> support;

          //--- This is a randomized algorithm, usually one should generate a random permutation ....
          //--- however we are lazy and leave this for the caller... stl have a function random_shuffle
          //--- Initialize algorithm

          vector3_iterator i = begin;
          compute_circumscribed_sphere(*i, minimal);

          support.m_cnt = 1;
          support.m_points[0] = (*i); ++i;

          while ( i != end )
          {
            if ( !support.contains( (*i) ) )
            {
              if ( !OpenTissue::intersect::point_sphere( (*i), minimal ) )
              {
                sphere_type sphere;
                switch(support.m_cnt)
                {
                case 1:            update_support1( (*i), support, sphere);          break;
                case 2:            update_support2( (*i), support, sphere);          break;
                case 3:            update_support3( (*i), support, sphere);          break;
                case 4:            update_support4( (*i), support, sphere);          break;
                default:                                                             break;
                }
                if ( sphere.radius() > minimal.radius() )
                {
                  minimal = sphere;
                  i = begin;
                  continue;
                }
              }
            }
            ++i;
          }
        }

      };

    } // namespace detail

    /**
    * Convenience function.
    *
    * @param begin
    * @param end
    * @param sphere
    */
    template<typename vector3_iterator, typename sphere_type>
    void compute_smallest_sphere(vector3_iterator begin, vector3_iterator end, sphere_type & sphere)
    {
      assert(begin!=end || !"compute_smallest_sphere(): begin was equal to end");

      detail::ComputeSmallestSphere()(begin,end,sphere);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SMALLEST_SPHERE_H
#endif
