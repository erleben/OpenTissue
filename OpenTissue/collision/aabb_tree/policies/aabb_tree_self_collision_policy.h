#ifndef OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_SELF_COLLISION_POLICY_H
#define OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_SELF_COLLISION_POLICY_H
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

    template<typename aabb_tree_geometry>
    class SelfCollisionPolicy
    {
    public:

      typedef typename aabb_tree_geometry::bvh_type           bvh_type;
      typedef typename bvh_type::volume_type                  volume_type;
      typedef typename bvh_type::geometry_type                geometry_type;
      typedef typename bvh_type::geometry_iterator            geometry_iterator;
      typedef typename bvh_type::bv_ptr                       bv_ptr;
      typedef typename bvh_type::annotated_bv_ptr             annotated_bv_ptr;
      typedef typename bvh_type::annotated_bv_type            annotated_bv_type;

      typedef typename volume_type::math_types                math_types;
      typedef typename math_types::real_type                  real_type;
      typedef typename math_types::vector3_type               vector3_type;
      typedef typename aabb_tree_geometry::vertex_data_type   vertex_data_type;

    protected:

      typedef std::list<std::pair<vertex_data_type*, vertex_data_type*> >   edge_container;
      typedef std::map<geometry_type *, edge_container >  edge_face_lut_type;
      edge_face_lut_type   m_edge_face_map;

    public:

      bool curvature_test(bv_ptr ){return false;}
      bool curvature_test(bv_ptr,bv_ptr){return false;}
      bool adjacent(bv_ptr,bv_ptr){return false;}

      bool overlap(bv_ptr A,bv_ptr B) {   return OpenTissue::intersect::aabb_aabb(A->volume(),B->volume());  }

      template<typename contact_point_container>
      void reset(contact_point_container & /*contacts*/)
      {
        //--- Make sure we are ready for doing book keeping, so we wont detect redudant contacts
        m_edge_face_map.clear();
      }

      template<typename contact_point_container>
      void report(
        bv_ptr bvA_
        , bv_ptr bvB_
        , contact_point_container & contacts
        )
      {
        annotated_bv_ptr bvA = boost::static_pointer_cast<annotated_bv_type>(bvA_);
        annotated_bv_ptr bvB = boost::static_pointer_cast<annotated_bv_type>(bvB_);

        geometry_type * A = &(*(bvA->geometry_begin()));
        geometry_type * B = &(*(bvB->geometry_begin()));

        if(A==B)
          return;
        if(is_sharing_node(A,B))
          return;
        compute_contacts(A,B,contacts);
      }

      bool is_sharing_node( geometry_type const * A, geometry_type const * B) const
      {
        if(A->m_p0 == B->m_p0)
          return true;
        if(A->m_p0 == B->m_p1)
          return true;
        if(A->m_p0 == B->m_p2)
          return true;

        if(A->m_p1 == B->m_p0)
          return true;
        if(A->m_p1 == B->m_p1)
          return true;
        if(A->m_p1 == B->m_p2)
          return true;

        if(A->m_p2 == B->m_p0)
          return true;
        if(A->m_p2 == B->m_p1)
          return true;
        if(A->m_p2 == B->m_p2)
          return true;

        return false;
      }

      template<typename contact_point_container>
      bool compute_contacts(
        geometry_type * A
        , geometry_type * B
        , contact_point_container & contacts
        )
      {
        bool collision = false;

        collision |= handle_edge_face_intersection( A->m_p0, A->m_p1, B, contacts );
        collision |= handle_edge_face_intersection( A->m_p1, A->m_p2, B, contacts );
        collision |= handle_edge_face_intersection( A->m_p2, A->m_p0, B, contacts );

        collision |= handle_edge_face_intersection( B->m_p0, B->m_p1, A, contacts );
        collision |= handle_edge_face_intersection( B->m_p1, B->m_p2, A, contacts );
        collision |= handle_edge_face_intersection( B->m_p2, B->m_p0, A, contacts );

        return collision;
      }

      template<typename contact_point_container>
      bool handle_edge_face_intersection(
        vertex_data_type * origin
        , vertex_data_type * destination
        , geometry_type * triangle
        , contact_point_container & contacts
        )
      {
        typedef typename contact_point_container::value_type   contact_point_type;

        using std::min;

        //--- At this point it should be verified that we have not
        //--- seen a contact before, between this edge and face.
        //
        if(exist_edge_face(origin,destination,triangle))
          return false;

        vector3_type O = origin->position();
        vector3_type D = destination->position();

        vector3_type x1 = triangle->m_p0->position();
        vector3_type x2 = triangle->m_p1->position();
        vector3_type x3 = triangle->m_p2->position();

        geometry::Plane<math_types> plane;
        plane.set(x1,x2,x3);

        real_type dO = plane.signed_distance(O);
        real_type dD = plane.signed_distance(D);

        if(dO>=0 && dD>=0)
          return false;
        if(dO<0 && dD<0)
          return false;

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

        vector3_type u;
        u = (D-O)*t + O;
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
        real_type lower = -delta;
        real_type upper = 1.+delta;
        if(
          (w1>lower)&&(w1<upper)
          &&
          (w2>lower)&&(w2<upper)
          &&
          (w3>lower)&&(w3<upper)
          )
        {

          m_edge_face_map[triangle].push_back( std::make_pair(origin,destination) );

          contact_point_type cp;

          cp.m_distance = min(dO,dD);
          cp.m_n = plane.n();
          cp.m_p = u;

          cp.m_A0 = origin;
          cp.m_A1 = destination;
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

          contacts.push_back(cp);

          return true;
        }
        return false;
      }

      bool exist_edge_face(
        vertex_data_type * origin
        , vertex_data_type * destination
        , geometry_type * triangle
        )
      {
        edge_container * edges = &m_edge_face_map[triangle];

        if(edges->empty())
          return false;

        typename edge_container::iterator edge = edges->begin();
        typename edge_container::iterator end  = edges->end();

        for(;edge!=end;++edge)
        {
          vertex_data_type * A = edge->first;
          vertex_data_type * B = edge->second;
          if(
            ((A==origin) && (B==destination))
            ||
            ((B==origin) && (A==destination))
            )
            return true;
        }
        return false;
      }

    };

  } // namespace aabb_tree
} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_SELF_COLLISION_POLICY_H
#endif
