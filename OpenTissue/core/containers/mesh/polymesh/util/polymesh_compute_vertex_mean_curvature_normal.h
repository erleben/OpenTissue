#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_MEAN_CURVATURE_NORMAL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_MEAN_CURVATURE_NORMAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_cot.h>
#include <OpenTissue/core/geometry/geometry_compute_area_mixed.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_boundary.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_vertex_neighbors_triangular.h>
#include <OpenTissue/core/math/math_precision.h>  // Needed for math::machine_precision

#include <cassert> // Needed for assert

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Compute Vertex Curvature Normal.
    * This implementation is based on the paper:
    *
    *    Meyer, M., Desbrun, M., Schrˆder, P., AND Barr, A. H. Discrete Differential Geometry Operators for Triangulated 2-Manifolds, 2002. VisMath.
    * 
    *
    * @param v         A reference to the vertex at which the curvature normal should be computed.
    * @param Kh        Upon return this argument holds the computed vertex curvature normal.
    *
    * @return          If computation were succesfully then the return value is true otherwise it is false.
    */
    template<typename mesh_type>
    bool compute_vertex_mean_curvature_normal(PolyMeshVertex<mesh_type> const & v, typename mesh_type::math_types::vector3_type & Kh)
    {
      typedef          PolyMeshVertex<mesh_type>               vertex_type;
      typedef typename mesh_type::vertex_iterator              vertex_iterator;
      typedef typename mesh_type::face_iterator                face_iterator;
      typedef typename mesh_type::vertex_face_circulator       vertex_face_circulator;
      typedef typename mesh_type::vertex_halfedge_circulator   vertex_halfedge_circulator;

      typedef typename mesh_type::math_types                   math_types;
      typedef typename math_types::vector3_type                vector3_type;
      typedef typename math_types::real_type                   real_type;

      static real_type const epsilon = math::machine_precision<real_type>();
      static real_type const zero    = real_type(); //--- by standard default constructed integral types is zero
      static real_type const two     = boost::numeric_cast<real_type>(2.0); 

      if(is_boundary(v))
      {
        std::cerr << "compute_vertex_mean_curvature_normal(): boundary vertex encountered" << std::endl;
        return false;
      }
      if(!is_vertex_neighbors_triangular(v))
      {
        std::cout << "compute_vertex_mean_curvature_normal(): non-triangular face encountered" << std::endl;
        return false;
      }

      real_type area = zero;
      Kh.clear();  // By OT convention should set all elements to zero.
      vector3_type p = v.m_coord;

      //using std::max;
      //using std::min;
      //{
      //  real_type one = 1.0;
      //  vertex_halfedge_circulator h(v),hend;
      //  for(;h!=hend;++h)
      //  {
      //    vector3_type nbr   = h->get_destination_iterator()->m_coord;
      //    vector3_type left  = h->get_twin_iterator()->get_prev_iterator()->get_origin_iterator()->m_coord;
      //    vector3_type right = h->get_next_iterator()->get_destination_iterator()->m_coord;
      //    real_type d_left  = unit(nbr-left)*unit(p-left);
      //    real_type d_right = unit(nbr-right)*unit(p-right);
      //    real_type a_left  = acos(    min(one, max(-one, d_left ))  );
      //    real_type a_right = acos(    min(one, max(-one, d_right))  );						
      //    real_type w = 1.0/tan(a_left) + 1.0/tan(a_right);						
      //    Kh += w * (p-nbr);						
      //  }
      //}
      //{
      //  vertex_halfedge_circulator h(v),hend;
      //  for(;h!=hend;++h)
      //  {
      //    vector3_type pi = h->get_next_iterator()->get_origin_iterator()->m_coord;
      //    vector3_type pj = h->get_next_iterator()->get_destination_iterator()->m_coord;
      //    area += compute_area_mixed(p,pi,pj);
      //  }
      //}
      //--- 
      //---  Orignally we have
      //--- 
      //---                  p                                                                                //
      //---                 /|\                                                                               //
      //---                / | \                                                                              //
      //---      alpha_k  /  |  \ beta_k        K_p =  1/A_p * sum_k  (cot(alpha_k)+cot(beta_k))*(p-p_k)      //
      //---               \  |  /                                                                             //
      //---                \ | /                                                                              //
      //---                 \|/                                                                               //
      //---                 p_k                                                                               //
      //--- 
      //---  But we could just as easily re-write this as
      //---         
      //---                  p                                                                                      //
      //---                 /|\                                                                                     //
      //---                / | \                                                                                    //
      //---      alpha_j  /  |  \ beta_i        K_p =  1/A_p * sum_i ( cot(alpha_j)(p-p_j) + cot(beta_i))*(p-p_i) ) //
      //---              /---+---\                                                                                  //
      //---            p_i        p_j                                                                               //
      //--- 
      //---  which is the forumula we have implemented below
      vertex_halfedge_circulator h(v),hend;
      for(;h!=hend;++h)
      {
        assert( h->get_next_iterator()->get_origin_iterator()->get_handle().get_idx()      != v.get_handle().get_idx() || !"compute_vertex_mean_curvature_normal(): illegal topology");
        assert( h->get_next_iterator()->get_destination_iterator()->get_handle().get_idx() != v.get_handle().get_idx() || !"compute_vertex_mean_curvature_normal(): illegal topology");
        vector3_type pi = h->get_next_iterator()->get_origin_iterator()->m_coord;
        vector3_type pj = h->get_next_iterator()->get_destination_iterator()->m_coord;
        area += OpenTissue::geometry::compute_area_mixed(p,pi,pj);
        real_type cotan_alpha = OpenTissue::geometry::cot( pi,  pj, p  );
        real_type cotan_beta  = OpenTissue::geometry::cot( pj,  p,  pi );
        Kh += (cotan_alpha*(p-pj)) + (cotan_beta*(p-pi));
      }
      if (area > epsilon) 
        Kh /= two*area;  //--- hmmm, I thinnk there is a bug here, it should read: Kh /= four*area;???
      else 
      {
        //std::cout << "compute_vertex_mean_curvature_normal(): zero area clamping" << std::endl;
        Kh.clear();
        return false;
      }
      if((Kh*Kh)<epsilon)
      {
        //std::cout << "compute_vertex_mean_curvature_normal(): too small curvature normal clamping" << std::endl;
        Kh.clear();
      }
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_MEAN_CURVATURE_NORMAL_H
#endif
