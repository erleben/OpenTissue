#ifndef T4MESH_COLLISION_POLICY_H
#define T4MESH_COLLISION_POLICY_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/intersect/intersect_aabb_aabb.h>
#include <OpenTissue/core/geometry/geometry_barycentric.h>
#include <boost/shared_ptr.hpp>

#include <vector>

template<typename bvh_type_>
class T4MeshCollisionPolicy
{
public:

  class result_type
  {
  public:
    unsigned int m_idx_A;
    unsigned int m_idx_B;
  };

public:

  typedef          bvh_type_                          bvh_type;
  typedef typename bvh_type::volume_type              volume_type;
  typedef typename bvh_type::geometry_type            geometry_type;
  typedef typename bvh_type::bv_ptr                   bv_ptr;
  typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;
  typedef typename bvh_type::annotated_bv_type        annotated_bv_type;

  typedef typename volume_type::math_types            math_types;
  typedef typename math_types::real_type              real_type;
  typedef typename math_types::vector3_type           vector3_type;


  typedef std::vector<vector3_type>                   coord_container;
  typedef std::vector<result_type>                    results_container;

protected:

  coord_container * m_coords_A;   ///< Pointer to coordiates of object A. Must be set by invocing set_coords_A(...) method.
  coord_container * m_coords_B;   ///< Pointer to coordiates of object B. Must be set by invocing set_coords_B(...) method.

public:

  void set_coords_A(coord_container & coords) { m_coords_A = &coords; };
  void set_coords_B(coord_container & coords) { m_coords_B = &coords; };

public:

  bool overlap(bv_ptr A,bv_ptr B)
  {
    return OpenTissue::intersect::aabb_aabb(A->volume(),B->volume());
  };

  void reset(results_container & results) { results.clear(); };


  void report(bv_ptr A,bv_ptr B,results_container & results)
  {
    annotated_bv_ptr bvA = boost::static_pointer_cast<annotated_bv_type>(A);
    annotated_bv_ptr bvB = boost::static_pointer_cast<annotated_bv_type>(B);

    geometry_type geoA = *(bvA->geometry_begin());

    unsigned int idx_i_A = geoA->i()->idx();
    unsigned int idx_j_A = geoA->j()->idx();
    unsigned int idx_k_A = geoA->k()->idx();
    unsigned int idx_m_A = geoA->m()->idx();

    vector3_type & i_A  = (*m_coords_A)[idx_i_A];
    vector3_type & j_A  = (*m_coords_A)[idx_j_A];
    vector3_type & k_A  = (*m_coords_A)[idx_k_A];
    vector3_type & m_A  = (*m_coords_A)[idx_m_A];

    geometry_type geoB = *(bvB->geometry_begin());

    unsigned int idx_i_B = geoB->i()->idx();
    unsigned int idx_j_B = geoB->j()->idx();
    unsigned int idx_k_B = geoB->k()->idx();
    unsigned int idx_m_B = geoB->m()->idx();

    vector3_type & i_B  = (*m_coords_B)[idx_i_B];
    vector3_type & j_B  = (*m_coords_B)[idx_j_B];
    vector3_type & k_B  = (*m_coords_B)[idx_k_B];
    vector3_type & m_B  = (*m_coords_B)[idx_m_B];

    real_type delta = 10e-5;
    real_type lower = - delta;
    real_type upper = 1.+ delta;
    real_type w1,w2,w3,w4;

    OpenTissue::geometry::barycentric_algebraic(i_A,j_A,k_A,m_A,i_B,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
    OpenTissue::geometry::barycentric_algebraic(i_A,j_A,k_A,m_A,j_B,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
    OpenTissue::geometry::barycentric_algebraic(i_A,j_A,k_A,m_A,k_B,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
    OpenTissue::geometry::barycentric_algebraic(i_A,j_A,k_A,m_A,m_B,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
    OpenTissue::geometry::barycentric_algebraic(i_B,j_B,k_B,m_B,i_A,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
    OpenTissue::geometry::barycentric_algebraic(i_B,j_B,k_B,m_B,j_A,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
    OpenTissue::geometry::barycentric_algebraic(i_B,j_B,k_B,m_B,k_A,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
    OpenTissue::geometry::barycentric_algebraic(i_B,j_B,k_B,m_B,m_A,w1,w2,w3,w4);
    if( (w1>lower)&&(w1<upper) && (w2>lower)&&(w2<upper) && (w3>lower)&&(w3<upper) && (w4>lower)&&(w4<upper) )
    {
      result_type result;
      result.m_idx_A = geoA->idx();
      result.m_idx_B = geoB->idx();
      results.push_back(result);
      return;
    }
  }

};


// T4MESH_COLLISION_POLICY_H
#endif
