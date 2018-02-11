#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_UTIL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_UTIL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_face_area.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_face_normal.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_edge_direction.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_vertex_edge_voronoi_plane.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_edge_face_voronoi_plane.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_vertex_mean_curvature_normal.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_vertex_gaussian_curvature.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_taubin_smooth.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_laplacian_smooth.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_distance.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_point_inside.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_boundary.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_convex.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_concave.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_convex_boundary.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_saddle_point.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_collinear.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_planar.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_manifold.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_neighbor.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_shared_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_sharing_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_get_face_vertices.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_vertex_neighbors_triangular.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_edge_flip.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_face_merge.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_face_split.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_face_subdivide.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_face_collapse.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_vertex_expand.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_intelligent_patcher.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_naive_patcher.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_reflex_convex_decomposition.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_triangulate.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_subdivide.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_make_sphere.h>

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_UTIL_H
#endif
