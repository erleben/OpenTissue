#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_LOCAL_TRIANGLE_FRAME_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_LOCAL_TRIANGLE_FRAME_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace geometry
  {

    template< typename vector3_type_ >
    class LocalTriangleFrame
    {
    public:

      typedef vector3_type_                      vector3_type;
      typedef typename vector3_type::value_type  real_type;

    protected:

      vector3_type m_v0;  ///< Vertices in CCW order
      vector3_type m_v1;
      vector3_type m_v2;
      vector3_type m_n;   ///< Triangle face normal

    protected:

      vector3_type m_a_vec;
      vector3_type m_b_vec;
      vector3_type m_h_vec;
      real_type m_a_norm_inv;
      real_type m_b_norm;
      real_type m_h_norm_inv;

      real_type m_a;   ///< The value of the triangle extent along the positive direction of the local r-axe.
      real_type m_b;   ///< The value of the triangle extent along the negative direction of the local r-axe.
      real_type m_h;   ///< The value of the triangle extent along the positive direction of the local s-axe.


      unsigned int m_offset;  ///< Cyclic permuation offset, used to match the cyclic of vertices given in wcs to corresponding vertices in local frame.

      vector3_type m_nv0; ///< Vertex normal of vertex v0 in local frame.
      vector3_type m_nv1;
      vector3_type m_nv2;
      vector3_type m_ne0; ///< Edge normal of edge running between v0 and v1 in local frame.
      vector3_type m_ne1;
      vector3_type m_ne2;


      vector3_type m_a_unit;  ///< Unit vector of a-vector (m_a_vec).
      vector3_type m_h_unit;  ///< Unit vector of h-vector (m_h_vec).

    public:

      real_type const & a() const    {      return m_a;    }
      real_type const & b() const    {      return m_b;    }
      real_type const & h() const    {      return m_h;    }
      vector3_type const & nv0() const    {      return m_nv0;    }
      vector3_type const & nv1() const    {      return m_nv1;    }
      vector3_type const & nv2() const    {      return m_nv2;    }
      vector3_type const & ne0() const    {      return m_ne0;    }
      vector3_type const & ne1() const    {      return m_ne1;    }
      vector3_type const & ne2() const    {      return m_ne2;    }
      vector3_type const & v0() const    {      return m_v0;    }
      vector3_type const & v1() const    {      return m_v1;    }
      vector3_type const & v2() const    {      return m_v2;    }
      vector3_type const & n() const    {      return m_n;    }
      vector3_type const & unit_a() const    {      return m_a_unit;    }
      vector3_type const & unit_h( )const    {      return m_h_unit;    }

    private:

      /**
      * Find Cyclic Permuation Order.
      *
      * Given three vertices in CCW order, this method performs a cyclic permuation
      * of their order, such that the longest edge of the triangle will be between
      * the first and second vertex
      *
      * The number of cyclic shift needed is stored in the member variable m_offset. The
      * permuted order is stored in the member variables m_v0, m_v1, m_v2.
      *
      * @param p0     First vertex in global frame.
      * @param p1     Second vertex in global frame.
      * @param p2     Third vertex in global frame.
      */
      void find_permutation( vector3_type const & p0,  vector3_type const & p1, vector3_type const & p2 )
      {
        real_type l0 = norm(p1-p0);
        real_type l1 = norm(p2-p1);
        real_type l2 = norm(p0-p2);
        if (l0>=l1 && l0>=l2)
        {
          m_offset = 0;
          m_v0 = p0;
          m_v1 = p1;
          m_v2 = p2;
        }
        else if (l1>=l0 && l1>=l2)
        {
          m_offset = 1;
          m_v0 = p1;
          m_v1 = p2;
          m_v2 = p0;
        }
        else if (l2>=l0 && l2>=l1)
        {
          m_offset = 2;
          m_v0 = p2;
          m_v1 = p0;
          m_v2 = p1;
        }
        else
        {
          assert(!"fuck nowhere to go!!!");
        }
      }

    public:
      /**
      * Transform Normal
      *
      * @param n    The normal given in global frame
      *
      *
      * @return     The corresponding normal in local frame.
      */
      vector3_type  xform_normal(vector3_type const & n)
      {
        //---
        //---  This method implements the following construct in an elegant way
        //---
        //---            matrix3x3_type RT(
        //---                 m_a_unit(0),m_a_unit(1),m_a_unit(2),
        //---                 m_h_unit(0),m_h_unit(1),m_h_unit(2),
        //---                 m_n(0), m_n(1), m_n(2)
        //---                 );
        //---
        //---    n' = RT*n;
        //---
        return vector3_type(  m_a_unit*n, m_h_unit*n, m_n*n);
      }


    public:

      /**
      *
      *
      * @param p0
      * @param p1
      * @param p2
      */
      void init( vector3_type const & p0,  vector3_type const & p1, vector3_type const & p2)
      {
        find_permutation(p0,p1,p2);


        vector3_type e0 = m_v1 - m_v0;
        vector3_type e1 = m_v2 - m_v1;
        vector3_type e2 = m_v0 - m_v2;
        m_n = unit ( e0 % e1 );

        //
        //  Layout of vectors....
        //
        //                  v2
        //                *
        //             /  ^
        //            /   |
        //           /    |
        //          /     |
        //         e2     h               e1
        //        /       |
        //       /        |
        //      /         |
        //    |/          |
        // v0 *<-----b----*--------------------a----------------------->*  v1
        //     -------------------------e0----------------------------->
        //
        m_b_vec = unit( e0 );
        m_b_norm = e2 * m_b_vec;
        m_b = std::fabs( m_b_norm );

        assert( m_b_norm < 0 ); //---- hmmm, what should this test be?
        m_b_vec *=  m_b_norm ;

        m_a_vec = e0 + m_b_vec ;
        m_a = std::sqrt(m_a_vec*m_a_vec);
        assert( m_a > 0 || !"LocalTriangleFrame::init(): a was non-positive");
        m_a_norm_inv = 1. / m_a;

        m_h_vec = m_b_vec - e2;
        m_h = sqrt(m_h_vec*m_h_vec);
        assert( m_h > 0 || !"LocalTriangleFrame::init(): h was non-positive");
        m_h_norm_inv = 1. / m_h;

        assert( !is_zero(m_a_vec) || !"LocalTriangleFrame::init(): a vector was zero");
        assert( !is_zero(m_h_vec) || !"LocalTriangleFrame::init(): h vector was zero");

        m_a_unit = unit(m_a_vec);
        m_h_unit = unit(m_h_vec);
      }


      /**
      * Get Local Triangle Coordinates.
      * Note initLocalCoordFrame must be invoked prior to invocation of this method.
      *
      * @param p   World space Coordinates.
      * @param u   Upon return this vector contains the local triangle
      *            coordinates r,s and t.
      */
      vector3_type get_local_coord( vector3_type const & p ) const
      {
        //---- Vector from local origo to point
        vector3_type pp = p - m_v0 + m_b_vec;
        return vector3_type (
          (pp * m_a_vec) * m_a_norm_inv,
          (pp * m_h_vec) * m_h_norm_inv,
          m_n*(p -m_v0)
          );
      }


      /**
      * Compute Local Normals.
      * Assumes that the init() method has been invoked prior to calling this method.
      * To access the local normals, use the nv0,nv1,nv2,ne0,ne1,ne2 accessor methods.
      *
      * Observe that the local face normal is simply the positive z-axis, so there
      * is no need for actual computing it.
      *
      * @param nv0       Vertex normal corresponding to vertex p0 in global frame.
      * @param nv1
      * @param nv2
      * @param ne0       Edge normal corresponding to edge running between p0 and p1 in global frame.
      * @param ne1
      * @param ne2
      */
      void compute_local_normals(
        vector3_type const & nv0
        , vector3_type const & nv1
        , vector3_type const & nv2
        , vector3_type const & ne0
        , vector3_type const & ne1
        , vector3_type const & ne2
        )
      {
        switch(m_offset)
        {
        case 0:
          m_nv0 = xform_normal(nv0);
          m_nv1 = xform_normal(nv1);
          m_nv2 = xform_normal(nv2);
          m_ne0 = xform_normal(ne0);
          m_ne1 = xform_normal(ne1);
          m_ne2 = xform_normal(ne2);
          break;
        case 1:
          m_nv0 = xform_normal(nv1);
          m_nv1 = xform_normal(nv2);
          m_nv2 = xform_normal(nv0);
          m_ne0 = xform_normal(ne1);
          m_ne1 = xform_normal(ne2);
          m_ne2 = xform_normal(ne0);
          break;
        case 2:
          m_nv0 = xform_normal(nv2);
          m_nv1 = xform_normal(nv0);
          m_nv2 = xform_normal(nv1);
          m_ne0 = xform_normal(ne2);
          m_ne1 = xform_normal(ne0);
          m_ne2 = xform_normal(ne1);
          break;
        default:
          assert(!"LocalTriangleFrame::compute_local_normals(...): Unexpected offset value encountered!");
          break;
        };
      }

    };

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_LOCAL_TRIANGLE_FRAME_H
#endif
