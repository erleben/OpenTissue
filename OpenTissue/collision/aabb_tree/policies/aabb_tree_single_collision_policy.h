#ifndef OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_SINGLE_COLLISION_POLICY_H
#define OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_SINGLE_COLLISION_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/intersect/intersect_aabb_aabb.h>

namespace OpenTissue
{
  namespace aabb_tree
  {

    /**
    * Single AABB Tree Collision Policy.
    * This collision policy have been tailored to the needs of particle/point against
    * a BVH query.
    */
    template<typename aabb_tree_geometry>
    class SingleCollisionPolicy
    {
    public:

      typedef typename aabb_tree_geometry::bvh_type       bvh_type;
      typedef typename bvh_type::volume_type              volume_type;
      typedef typename bvh_type::geometry_type            geometry_type;
      typedef typename bvh_type::geometry_iterator        geometry_iterator;
      typedef typename bvh_type::bv_ptr                   bv_ptr;
      typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;
      typedef typename bvh_type::annotated_bv_type        annotated_bv_type;

      typedef typename volume_type::math_types            math_types;
      typedef typename math_types::real_type              real_type;
      typedef typename math_types::vector3_type          vector3_type;

    public:

      template<typename contact_point_container>
      void reset(contact_point_container & /*contacts*/) {  }


      /**
      *
      * @param point    The point/particle data type that should be tested for collision
      *                 with the BVH. The data type must support the two methods position()
      *                 and old_position().
      */
      template<typename coord_transform,typename point_data_type>
      bool overlap(
        coord_transform const & /*xform*/
        , bv_ptr bv
        , point_data_type const & point 
        )
      {
        vector3_type r     = point.position();
        vector3_type r_old = point.old_position();

        vector3_type m = vector3_type( math::detail::highest<real_type>() );
        vector3_type M = vector3_type( math::detail::lowest<real_type>() );

        m = min(m,r);
        m = min(m,r_old);
        M = max(M,r);
        M = max(M,r_old);

        volume_type sweeping(m,M);

        return OpenTissue::intersect::aabb_aabb(sweeping, bv->volume());
      }

      /**
      *
      * @param point    The point/particle data type that should be tested for collision
      *                 with the BVH. The data type must support the two methods position()
      *                 and old_position().
      */
      template<typename coord_transform,typename contact_point_container,typename point_data_type>
      void report(
        coord_transform const & /*xform*/
        , bv_ptr bv
        , point_data_type const & point
        , contact_point_container & contacts
        )
      {
        typedef typename contact_point_container::value_type   contact_point_type;

        using std::fabs;

        vector3_type r     = point.position();
        vector3_type r_old = point.old_position();

        annotated_bv_ptr annotated_bv = boost::static_pointer_cast<annotated_bv_type>(bv);

        geometry_type * triangle = &(*( annotated_bv->geometry_begin()));
        assert(triangle);

        vector3_type x1 = triangle->m_p0->position();
        vector3_type x2 = triangle->m_p1->position();
        vector3_type x3 = triangle->m_p2->position();

        geometry::Plane<math_types> plane;
        plane.set(x1,x2,x3);

        real_type dO = plane.signed_distance(r_old);
        real_type dD = plane.signed_distance(r);

        real_type t;
        //---
        //--- du = t*dD + (1-t)dO
        //--- du = t*dD + dO - t*dO     =>   u = (D-O)*t + O
        //--- t = (du-dO)/(dD-dO)
        //---
        //--- now du = 0 so
        //---
        //--- t = -dO/(dD-dO) = dO/(dO-dD)
        //---
        t = dO/(dO-dD);

        static real_type threshold = math::working_precision<real_type>();
        if((t<-threshold) || (t>(1.+threshold)))
          return;

        vector3_type u;
        u = (r- r_old)*t + r_old;

        //--- Edge e10 of A intersect face plane of B, now we must
        //--- find out if the intersection point u lies inside the
        //--- perimeter of face B, we do this by computing the barycentric
        //--- coordinates of u.
        real_type w1,w2,w3;
        OpenTissue::geometry::barycentric_algebraic(x1,x2,x3,u,w1,w2,w3);

        //--- Now we can finally see wheter the intersection point is inside the
        //--- triangle perimeter of B, by looking at the values of the barycentric
        //--- coordinates

        real_type delta = 10e-5;
        real_type lower = - delta;
        real_type upper = 1.+ delta;
        if(
          (w1>lower)&&(w1<upper)
          &&
          (w2>lower)&&(w2<upper)
          &&
          (w3>lower)&&(w3<upper)
          )
        {
          contact_point_type cp;

          cp.m_distance = fabs(dD);
          cp.m_n = plane.n();
          cp.m_p = u;

          cp.m_A0 = const_cast<point_data_type*>(&point);
          cp.m_A1 = 0;
          cp.m_A2 = 0;

          cp.m_B0 = triangle->m_p0;
          cp.m_B1 = triangle->m_p1;
          cp.m_B2 = triangle->m_p2;

          cp.m_a0 = t;
          cp.m_a1 = 0;
          cp.m_a2 = 0;

          cp.m_b0 = w1;
          cp.m_b1 = w2;
          cp.m_b2 = w3;

          contacts.push_back( cp );
        }

      }

    };

  } // namespace aabb_tree

} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_SINGLE_COLLISION_POLICY_H
#endif
