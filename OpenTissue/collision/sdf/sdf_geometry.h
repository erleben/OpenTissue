#ifndef OPENTISSUE_COLLISION_SDF_SDF_GEOMETRY_H
#define OPENTISSUE_COLLISION_SDF_SDF_GEOMETRY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


#include <OpenTissue/collision/bvh/bvh.h>
#include <OpenTissue/collision/collision_geometry_interface.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_compute_obb_aabb.h>
#include <OpenTissue/utility/utility_class_id.h>


#include <list>

namespace OpenTissue
{
  namespace sdf
  {



    /**
    * Signed Distance Map Geometry.
    * This is infact a double representation, a signed distance map is used
    * and a point sampling of the surface of the object is also used.
    *
    * When doing collision detection between two bodies one simply looks up
    * the point sampling in the distance map of the other body and vice versa.
    * All sample points with distance less than collision envelope generates a
    * contact point. The separation (or penetration) distance is simply the
    * signed distance value at the sample points position, the contact normal
    * is the gradient of the signed distance map the sample position, and the
    * location of the contact points is given by the sample position.
    */
    template<typename mesh_type_,typename grid_type_>
    class Geometry
      : public OpenTissue::collision::GeometryInterface< typename mesh_type_::math_types >
      , public OpenTissue::utility::ClassID< Geometry<mesh_type_, grid_type_ > >
    {
    public:

      typedef          mesh_type_                            mesh_type;
      typedef          grid_type_                            grid_type;
      typedef typename mesh_type::math_types                 math_types;

      typedef typename math_types::real_type                 real_type;
      typedef typename math_types::vector3_type              vector3_type;
      typedef typename math_types::matrix3x3_type            matrix3x3_type;

      typedef          geometry::Sphere<math_types>          sphere_type;
      typedef typename std::list<vector3_type>               point_container;
      typedef typename point_container::iterator             point_iterator;
      typedef typename point_container::const_iterator       const_point_iterator;

      typedef          bvh::BoundingVolumeHierarchy<sphere_type,vector3_type*>   bvh_type;

    public: //--- kenny 20060420: Maybe these should be protected?

      mesh_type             m_mesh;            ///< A pointer to the mesh representing the zero-level set surface of the signed distance field.
      grid_type             m_phi;             ///< A pointer to a map containing the signed distance field.
      real_type             m_max_radius;      ///< The maximum radius
      vector3_type          m_min_coord;       ///< The AABB corner node with smallest coordinates (in BF coords).
      vector3_type          m_max_coord;       ///< The AABB corner node with largest coordinates (in BF coords).
      point_container       m_sampling;        ///< A point sampling, representing the zero level set surface of the signed distance map.
      bvh_type              m_bvh;             ///< Sphere BVH (of sample points), used to speed up collision queries.

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::sdf::Geometry<mesh_type_,grid_type_> >::class_id(); }

      Geometry() 
        : m_max_radius() 
        , m_min_coord(0,0,0)
        , m_max_coord(0,0,0)
      {}

    public:

      /**
      * Get Half Side Extents
      *
      * @return          A 3D vector, where each component corresponds to the
      *                  half-side extent of the tight fitting OBB of the signed
      *                  distance field.
      */
      vector3_type ext() const { return vector3_type ( 0.5*(m_max_coord - m_min_coord) ); }

      /**
      * Maximum Radius.
      * Return a radius value equal to or greater than the radius of the minimum
      * enclosing sphere (with center at center of mass).
      *
      * @return          The maximum radius.
      */
      real_type const & max_radius() const { return m_max_radius; }

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
        OpenTissue::geometry::compute_obb_aabb(r,R,this->ext(),min_coord,max_coord);
      }

    };

  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_GEOMETRY_H
#endif
