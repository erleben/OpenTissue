#ifndef CURVATURE_MESH_COLLISION_POLICY_H
#define CURVATURE_MESH_COLLISION_POLICY_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh.h>
#include <OpenTissue/collision/intersect/intersect_aabb_aabb.h>

#include <OpenTissue/core/geometry/geometry_triangle.h>
#include <OpenTissue/collision/intersect/intersect_triangle_triangle.h>

#include <boost/shared_ptr.hpp>

template<typename bvh_type_>
class CurvatureMeshCollisionPolicy
{
public:

  typedef          bvh_type_                          bvh_type;
  typedef typename bvh_type::volume_type              volume_type;
  typedef typename bvh_type::geometry_type            geometry_type;
  typedef typename bvh_type::bv_type                  bv_type;
  typedef typename bvh_type::bv_ptr                   bv_ptr;
  typedef typename bvh_type::annotated_bv_type        annotated_bv_type;
  typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;

  typedef typename volume_type::math_types            math_types;
  typedef typename math_types::real_type              real_type;
  typedef typename math_types::vector3_type           vector3_type;

  typedef typename OpenTissue::polymesh::PolyMesh<>   mesh_type;
  typedef typename mesh_type::halfedge_type           halfedge_type;
  typedef typename mesh_type::face_type               face_type;
  typedef typename bv_type::adjacency_container       adjacency_container;
  typedef typename bv_type::adjacency_iterator        adjacency_iterator;

public:

  class ResultType
  {
  public:
    face_type *  m_face_A;
    face_type *  m_face_B;
  };

  typedef ResultType                                  result_type;
  typedef std::vector<result_type>                    results_container;

public:

  bool curvature_test(bv_ptr bv) { return (bv->m_dir_lut!=0); }
  bool curvature_test(bv_ptr A, bv_ptr B)  {  return (A->m_dir_lut & B->m_dir_lut)!=0;  }
  bool overlap(bv_ptr A,bv_ptr B) {   return OpenTissue::intersect::aabb_aabb(A->volume(),B->volume());  }
  bool adjacent(bv_ptr A, bv_ptr B)
  {
    if(A->is_root() || B->is_root())
      return true;
    assert(A->m_adjacency.size()>0);
    assert(B->m_adjacency.size()>0);
    if(B->m_adjacency.size()<A->m_adjacency.size())
    {
      bv_ptr tmp = B;
      B = A;
      A = tmp;
    }
    for(adjacency_iterator a = A->m_adjacency.begin();a!=A->m_adjacency.end();++a)
    {
      for(adjacency_iterator b = B->m_adjacency.begin();b!=B->m_adjacency.end();++b)
      {
        if( (*a)==(*b) )
          return true;
      }
    }
    return false;
  }

  void reset(results_container & results){results.clear();}

  void report(bv_ptr A,bv_ptr B,results_container & results)
  {
    //--- This is terrible in-efficient....but it serves for demonstration purpose

    annotated_bv_ptr bvA = boost::static_pointer_cast<annotated_bv_type>(A);
    annotated_bv_ptr bvB = boost::static_pointer_cast<annotated_bv_type>(B);

    face_type * face_A = *(bvA->geometry_begin());
    face_type * face_B = *(bvB->geometry_begin());

    mesh_type::face_vertex_circulator vA(*face_A);
    vector3_type * a0 = &(vA->m_coord);++vA;
    vector3_type * a1 = &(vA->m_coord);++vA;
    vector3_type * a2 = &(vA->m_coord);

    mesh_type::face_vertex_circulator vB(*face_B);
    vector3_type * b0 = &(vB->m_coord);++vB;
    vector3_type * b1 = &(vB->m_coord);++vB;
    vector3_type * b2 = &(vB->m_coord);

    OpenTissue::geometry::Triangle<math_types> TA, TB;
    TA.bind(a0,a1,a2);
    TB.bind(b0,b1,b2);
    std::vector< vector3_type > points;
    if(OpenTissue::intersect::triangle_triangle(TA,TB,points))
    {
      result_type result;
      result.m_face_A = face_A;
      result.m_face_B = face_B;
      results.push_back(result);
    }
  }
};

// CURVATURE_MESH_COLLISION_POLICY_H
#endif
