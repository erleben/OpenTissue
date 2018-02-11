#ifndef OPENTISSUE_COLLISION_SDF_SDF_COLLISION_POLICY_H
#define OPENTISSUE_COLLISION_SDF_SDF_COLLISION_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_enclosing_indices.h>
#include <OpenTissue/core/containers/grid/util/grid_gradient_at_point.h>
#include <OpenTissue/core/containers/grid/util/grid_value_at_point.h>

namespace OpenTissue
{
  namespace sdf
  {


    /**
    * Signed Distance Field Collision Policy.
    */
    template <typename bvh_type_, typename coordsys_type_>
    class CollisionPolicy
    {
    public:

      typedef          bvh_type_                           bvh_type;
      typedef          coordsys_type_                      coordsys_type;

      typedef typename bvh_type::bv_ptr                    bv_ptr;
      typedef typename bvh_type::bv_ptr_container          bv_ptr_container;
      typedef typename bvh_type::bv_iterator               bv_iterator;

      typedef typename coordsys_type::vector3_type         vector3_type;
      typedef typename coordsys_type::value_type           real_type;


    public:

      real_type       m_envelope;       ///< Collision envelope, should be set prior to collision query.
      bool            m_flipped;        ///< This boolean flag indicates whether normal directions on
      ///< generated contact points should be flipped or not.
      coordsys_type   m_wcs_xform;      ///< Coordinate transform, brings a contact point from the local
      ///< model frame of the geometry type into the world coordinate
      ///< system.

    public:

      CollisionPolicy()
        : m_envelope(0.01)
        , m_flipped (false)
      {}

    public:

      /**
      * Contact Normal Flipped.
      * The caller may have reversed the roles of objects A and B, in which case contact
      * normals should be flipped when reporting them.
      *
      * If object roles are reveres then flipped should be set to true. Default value is false.
      *
      *
      * @return        The size of the collision envelope.
      */
      bool & flipped() { return m_flipped; }
      bool const & flipped() const { return m_flipped; }



      /**
      * Collision Envelope.
      * The size of the collision envelope dicates a threshold value, whenever objects
      * are within the collision envelope of each other then the policy will report
      * a collision.
      *
      * @return        The size of the collision envelope.
      */
      real_type & envelope() { return m_envelope; }
      real_type const & envelope() const { return m_envelope; }

      /**
      * World Coordinate System Transform.
      * The policy uses this transfrom to convert generated contact points into the correct frame.
      *
      * @return    A reference to the world coordinate system transform.
      */
      coordsys_type & wcs_xform() { return m_wcs_xform; }
      coordsys_type const & wcs_xform() const { return m_wcs_xform; }

    public:

      /**
      * Reset.
      * This method is invoked prior to performing any collision query.
      */
      template<typename contact_point_container>
      void reset(contact_point_container & /*contacts*/)    {     /*  contacts.clear(); */   }

      /**
      * Overlap Testing.
      *
      * @param xform          A coordinate transform that takes the geometry of the bv
      *                       into the local model frame of the sdf geometry.
      * @parma bv             A pointer to a bounding volume (bv) node.
      * @param geometry       A signed distance field geometry.
      *
      * @return               If the geometry of the bv (a sphere) is overlapping with
      *                       the zero-level set of the signed distance field geometry
      *                       then the return value it true otherwise it is false.
      */
      template<typename sdf_geometry_type>
      bool overlap( 
        coordsys_type const & xform
        , bv_ptr const & bv
        , sdf_geometry_type const & geometry 
        )
      {
        typedef typename coordsys_type::vector3_type    vector3_type;
        typedef typename coordsys_type::value_type      real_type;
        typedef typename sdf_geometry_type::grid_type    grid_type;

        grid_type const & phi = geometry.m_phi;

        vector3_type center = bv->volume().center();
        real_type radius = bv->volume().radius();
        xform.xform_point(center);
        real_type tst = radius + m_envelope;

        if(phi.min_coord() <= center &&  center <= phi.max_coord())
        {
          //--- Sphere center inside map grid, test to see if sphere
          //--- surface crosses zero-level set
          size_t i0,j0,k0,i1,j1,k1;
          OpenTissue::grid::enclosing_indices(phi, center, i0, j0, k0, i1, j1, k1);

          if(phi(i0,j0,k0) < tst)
            return true;
          if(phi(i0,j0,k1) < tst)
            return true;
          if(phi(i0,j1,k0) < tst)
            return true;
          if(phi(i0,j1,k1) < tst)
            return true;
          if(phi(i1,j0,k0) < tst)
            return true;
          if(phi(i1,j0,k1) < tst)
            return true;
          if(phi(i1,j1,k0) < tst)
            return true;
          if(phi(i1,j1,k1) < tst)
            return true;
          return false;
        }

        //--- Sphere center is outside map, see if sphere surface intersect
        //--- minimum AABB of sampling points
        vector3_type ext = geometry.ext();
        vector3_type proj;
        for(unsigned int i =0;i<3;++i)
        {
          if(ext(i)<center(i))
            proj(i) = ext(i);
          else if(center(i)<-ext(i))
            proj(i) = -ext(i);
          else
            proj(i) = center(i);
        }
        real_type sqr_tst = tst*tst;
        vector3_type diff = center - proj;
        if((diff*diff) < sqr_tst)
          return true;
        return false;
      }


      /**
      * Contact Point Reporting.
      *
      * @param xform                Coordinate transform, can be used to bring bv into model frame of geometry.
      * @param bv                   The bv (bounding volume) from  object A.
      * @param geometry             The geometry of object B.
      * @param contacts             Upon return any contact point is added to this container. Note
      *                             that the contact consist of a contact point, a contact normal
      *                             and a separation/penetration distance. These are specified in the
      *                             local coordinate system of the signed distance field geometry. The
      *                             caller is responsible for converting these into the world coordinate
      *                             system (WCS) if needed.
      *
      *                             Note that by convention the contact normal is always pointing from
      *                             the signed distance field geometry towards the bv geometry. That
      *                             is normals are pointing from A towards B by convention.
      *
      */
      template<typename sdf_geometry_type,typename contact_point_container>
      void report( 
        coordsys_type const & xform
        , bv_ptr const & bv
        , sdf_geometry_type const & geometry
        , contact_point_container & contacts 
        )
      {
        using std::min;
        typedef typename sdf_geometry_type::grid_type           grid_type;
        typedef typename contact_point_container::value_type   contact_point_type;

        grid_type const & phi = geometry.m_phi;

        vector3_type center = bv->volume().center();
        xform.xform_point(center);

        if(phi.min_coord() <= center &&  center <= phi.max_coord())
        {
          //--- sampling point inside map grid
          size_t i0,j0,k0,i1,j1,k1;
          OpenTissue::grid::enclosing_indices(phi, center, i0, j0, k0, i1, j1, k1);

          real_type d_min = math::detail::highest<real_type>();

          real_type d000 = phi(i0,j0,k0);
          real_type d001 = phi(i1,j0,k0);
          real_type d010 = phi(i0,j1,k0);
          real_type d011 = phi(i1,j1,k0);
          real_type d100 = phi(i0,j0,k1);
          real_type d101 = phi(i1,j0,k1);
          real_type d110 = phi(i0,j1,k1);
          real_type d111 = phi(i1,j1,k1);

          d_min = min(d_min,d000);
          d_min = min(d_min,d001);
          d_min = min(d_min,d010);
          d_min = min(d_min,d011);
          d_min = min(d_min,d100);
          d_min = min(d_min,d101);
          d_min = min(d_min,d110);
          d_min = min(d_min,d111);

          if(d_min >  m_envelope)
            return;

          real_type s = ( center(0) - ( i0 * phi.dx() + phi.min_coord(0) ) ) / phi.dx();
          real_type t = ( center(1) - ( j0 * phi.dy() + phi.min_coord(1) ) ) / phi.dy();
          real_type u = ( center(2) - ( k0 * phi.dz() + phi.min_coord(2) ) ) / phi.dz();

          real_type x00 = ( 1 - s ) * d000 + s * d001;
          real_type x01 = ( 1 - s ) * d010 + s * d011;
          real_type x10 = ( 1 - s ) * d100 + s * d101;
          real_type x11 = ( 1 - s ) * d110 + s * d111;
          real_type y0 = ( 1 - t ) * x00 + t * x01;
          real_type y1 = ( 1 - t ) * x10 + t * x11;
          real_type distance = ( 1 -u ) * y0 + u * y1;

          if(distance>m_envelope)
            return;

          vector3_type n = OpenTissue::grid::gradient_at_point(phi, center);
          if ( n(0) == phi.unused() )
            return;

          //--- NOTE: n is pointing from geometry towards bvh (from B to A)
          //---
          //--- by convention we want n to point from A to b.
          n = -unit(n); 

          //--- It may be that caller have reversed the roles of A and
          //--- B, if so we must flip normal otherwise caller will get
          //--- something strange back!
          if(m_flipped)
            n = - n;

          //--- Collision normal is computed in the local model frame of
          //--- the sdf_geometry_type. Caller expects collision points in
          //--- WCS, so we must transform contact normal and contact point
          //--- back into WCS.
          m_wcs_xform.xform_vector( n );
          m_wcs_xform.xform_point( center );

          //--- Finally we can create and return a contact point.
          contact_point_type cp;        
          cp.m_n = n;
          cp.m_p = center;
          cp.m_distance = distance;
          contacts.push_back(cp);
        }
      }

    };

  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_COLLISION_POLICY_H
#endif
