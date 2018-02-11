#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_GAUSSIAN_CURVATURE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_GAUSSIAN_CURVATURE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_is_angle_obtuse.h>
#include <OpenTissue/core/geometry/geometry_is_triangle_obtuse.h>
#include <OpenTissue/core/geometry/geometry_angle_from_cot.h>
#include <OpenTissue/core/geometry/geometry_compute_area_mixed.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_boundary.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_vertex_neighbors_triangular.h>

#include <OpenTissue/core/math/math_precision.h>  // Needed for math::machine_precision
#include <OpenTissue/core/math/math_constants.h>

#include <cassert> // Needed for assert

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Compute Gaussian Curvature of Vertex.
    * This implementation is based on the paper:
    *
    *    Meyer, M., Desbrun, M., Schröder, P., AND Barr, A. H. Discrete Differential Geometry Operators for Triangulated 2-Manifolds, 2002. VisMath.
    * 
    *
    * @param v         A reference to the vertex at which the Gaussian curvature should be computed.
    * @param Kg        Upon return this argument holds the computed Gaussian curvature.
    *
    * @return          If computation were succesfully then the return value is true otherwise it is false.
    */
    template<typename mesh_type, typename real_type2>
    bool compute_vertex_gaussian_curvature( PolyMeshVertex<mesh_type> const & v, real_type2 & Kg)
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
        std::cerr << "compute_vertex_gaussian_curvature(): boundary vertex encountered" << std::endl;
        return false;
      }
      if(!is_vertex_neighbors_triangular(v))
      {
        std::cout << "compute_vertex_gaussian_curvature(): non-triangular face encountered" << std::endl;
        return false;
      }
      real_type         area = zero;
      real_type    angle_sum = zero;
      vector3_type         p = v.m_coord;
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
      //---                  p                                                                                         //
      //---                 /|\                                                                                        //
      //---                / | \                                                                                       //
      //---      alpha_j  /  |  \ beta_i        K_p =  1/A_p * sum_i ( cot(alpha_j)(p-p_j) + cot(beta_i))*(p-p_i) )    //
      //---              /-------\                                                                                     //
      //---            p_i        p_j                                                                                  //
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
        //--- 
        //--- we could have used the cosine formula
        //---
        //---                     (pi-p) \cdot (pj-p) 
        //---     \cos(\theta) = ----------------------
        //---                     ||pi-p|| || pj-p || 
        //---
        //--- However, this involves computing the length of the vectors (pi-p) and (pj-p)
        //---
        //--- The angle_from_coton avoids this (no square root computations), at the cost of a atan2 call.
        //--- 
        angle_sum += OpenTissue::geometry::angle_from_cot(p,pi,pj);
      }
      if (area > epsilon) 
        Kg = boost::numeric_cast<real_type2> ( (two*math::detail::pi<real_type>() - angle_sum)/area  );
      else
        Kg = boost::numeric_cast<real_type2>( zero );
      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VERTEX_GAUSSIAN_CURVATURE_H
#endif
