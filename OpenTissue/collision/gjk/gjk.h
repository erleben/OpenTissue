#ifndef OPENTISSUE_COLLISION_GJK_GJK_H
#define OPENTISSUE_COLLISION_GJK_GJK_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

// GJK library routines
#include <OpenTissue/collision/gjk/gjk_compute_closest_points.h>
#include <OpenTissue/collision/gjk/gjk_constants.h>
#include <OpenTissue/collision/gjk/gjk_outside_edge_face_voronoi_plane.h>
#include <OpenTissue/collision/gjk/gjk_outside_triangle.h>
#include <OpenTissue/collision/gjk/gjk_outside_vertex_edge_voronoi_plane.h>
#include <OpenTissue/collision/gjk/gjk_reduce_edge.h>
#include <OpenTissue/collision/gjk/gjk_reduce_tetrahedron.h>
#include <OpenTissue/collision/gjk/gjk_reduce_triangle.h>
#include <OpenTissue/collision/gjk/gjk_signed_distance_to_edge_face_voronoi_plane.h>
#include <OpenTissue/collision/gjk/gjk_signed_distance_to_triangle.h>
#include <OpenTissue/collision/gjk/gjk_signed_distance_to_vertex_edge_voronoi_plane.h>
#include <OpenTissue/collision/gjk/gjk_simplex.h>
#include <OpenTissue/collision/gjk/gjk_support_functors.h>
#include <OpenTissue/collision/gjk/gjk_voronoi_simplex_solver_policy.h>

// Below is the old obsolete GJK implementation



#include <boost/cast.hpp> //--- needed for boost::numeric_cast

namespace OpenTissue
{
  namespace gjk
  {
    namespace obsolete
    {
      namespace detail
      {
        /**
        * GJK Algorithm.
        * This class implements the Gilbert, Johnson and Keerthi
        * collision detection algorithm, also known as GJK.
        *
        * The algorithm is an iterative simplex based algorithm, our
        * implementation is based on the original code implemented by
        * Gino van den Bergen in his GJK-engine (which later has become
        * part of his SOLID collision detection library), thanks Gino:-)
        *
        * For details on GJK, we refer to
        *
        *   [1] Uffe ??? and Niels ???:
        *       ???.
        *       DIKU master thesis no. 01-01-2, pp. 49-77
        *   [2] Gino van den Bergen:
        *       A Fast and Robust GJK Implementation for Collision Detection of Convex Objects.
        *       Journal of Graphics Tools, 4(2):7-25 (1999).
        *
        * In [1] there is an excellent explanation and proof of the
        * sub-algorithm in GJK, it also contains a new method for
        * dealing with numerical errors. The authors claim their method
        * is more robust than Gino's. We have not implemented it, but
        * sugest it as a possible resolution in case we ever get into
        * numerical problems with GJK.
        *
        * Support Mapping Function
        *
        *  p = get_support_point(v)
        *
        * This method is needed by GJK. The method computes
        * the extremum point in a given direction of a convex
        * point set.
        *
        * @param v      A pointer to a vector, holding the direction
        * @return       A pointer to a vector, which upon return should
        *               contain the extrenum point in the specified
        *               direction.
        */
        template<typename vector3_type_>
        class GJK
        {
        public:

          typedef          vector3_type_               vector3_type;
          typedef typename vector3_type::value_traits  value_traits;
          typedef typename vector3_type::value_type    real_type;

        protected:

          real_type m_rel_error;     ///< Relative error in the computed distance
          real_type m_abs_error;     ///< Absolute error if the distance is almost zero
          real_type m_abs_error2;    ///< Absolute squared error

          vector3_type m_p[4];       ///< Support points of object A
          vector3_type m_q[4];       ///< Support points of object B
          vector3_type m_y[4];       ///< Support points of A - B

          int m_bits;                ///< Identifies current simplex
          int m_last;                ///< Identifies last found support point
          int m_last_bit;            ///< last_bit = 1<<last
          int m_all_bits;            ///< all_bits = bits|last_bit

          real_type m_det[16][4];    ///< Cached sub-determinants
          real_type m_dp[4][4];      ///< Cached dot products

        public:

          GJK()
            : m_rel_error(  boost::numeric_cast<real_type>(1e-6)  )
            , m_abs_error(  boost::numeric_cast<real_type>(1e-10) )
            , m_abs_error2( boost::numeric_cast<real_type>(1e-20) ) 
          {}

        public:

          /**
          * Intersection Query.
          * This is the ISA-GJK version.
          * Note if coherence should be exploited one should surply the
          * method with the separation axe computed from the previous
          * iteration. The initial value could be any arbitary vector, we
          * sugest that you use the difference between the centers of A and B.
          *
          * @param A       A pointer to one convex volume
          * @param B       A pointer to another convex volume
          * @param g       A pointer to a vector, which contains an
          *                inital guess for a separation axe, between
          *                A and B. Upon return the vector will contain
          *                the separation axe, computed by GJK.
          *
          * @return        If A and B are intersecting then the return
          *                value is true otherwise it is false.
          */
          template<typename convex_shape_type1, typename convex_shape_type2>
          bool is_intersecting(
            convex_shape_type1 & A
            , convex_shape_type2 & B
            , vector3_type & g
            )
          {
            m_bits = 0;
            m_all_bits = 0;
            do
            {
              //--- Determine a position in y, which is free
              //--- and can be used for storing the new support
              //--- point in.
              m_last = 0;
              m_last_bit = 1;

              while(m_bits & m_last_bit)//--- Loop until we find an entry not used in the current simplex
              {
                ++m_last;
                m_last_bit <<= 1;
              }
              //--- Get new support point
              vector3_type wA = A.get_support_point(-g);
              vector3_type wB = B.get_support_point(g);
              vector3_type w = wA - wB;
              //--- Separation Axe test
              //--- Provides fast rejection for non-overlapping objects
              if( g*w > 0)
              {
                return false;
              }
              if(this->is_degenerate(w))
              {
                return false;
              }
              m_y[m_last] = w;
              m_all_bits = m_bits | m_last_bit;
              if(!this->get_closest(g))
              {
                return false;
              }
              //--- Note if bits == 15 then we have a tetrahedron as
              //--- our simplex (we could not reduce it in the sub-algorithm), since
              //--- the closest point must be orthogonal to all edges of the
              //--- simplex, this means origo must be inside the simplex i.e. we
              //--- have an intersection
              //---
            }
            while(m_bits < 15 && !this->is_approx_zero(g));

            return true;
          }

          /**
          * Common Point Queury.
          * In case of intersection a common point between the two
          * specified convex volumes is computed.
          *
          * Note both pa and pb will contain the same value, this
          * is because we assume that A and B are represented in
          * the same coordinate frame. GJK could take affine
          * transformations of A and B into account. Meaning that
          * A and B could be given in their local frame, but we
          * have no need for this!!!
          *
          * @param A       A pointer to one convex volume
          * @param B       A pointer to another convex volume
          * @param g       A pointer to a vector, which contains an
          *                inital guess for a separation axe, between
          *                A and B. Upon return the vector will contain
          *                the separation axe, computed by GJK.
          * @param pa      A pointer to a vector, which upon return
          *                contains a closest point on A to B
          * @param pb      A pointer to a vector, which upon return
          *                contains a closest point on B to A
          *
          * @return        If A and B are intersecting then the return
          *                value is true otherwise it is false.
          */
          template<typename convex_shape_type1, typename convex_shape_type2>
          bool get_common_point(
            convex_shape_type1 & A
            , convex_shape_type2 & B
            , vector3_type & g
            , vector3_type & pa
            , vector3_type & pb
            )
          {
            m_bits = 0;
            m_all_bits = 0;
            do
            {
              //--- Determine a position in y, which is free
              //--- and can be used for storing the new support
              //--- point in.
              m_last = 0;
              m_last_bit = 1;
              while(m_bits & m_last_bit)//--- Loop until we find an entry not used in the current simplex
              {
                ++m_last;
                m_last_bit <<= 1;
              }

              m_p[m_last] = A.get_support_point(-g);
              m_q[m_last] = B.get_support_point(g);
              vector3_type w = m_p[m_last] - m_q[m_last];
              if(g * w > 0)
              {
                return false;
              }
              if(this->is_degenerate(w))
              {
                return false;
              }
              m_y[m_last] = w;
              m_all_bits = m_bits | m_last_bit;
              if(!this->get_closest(g))
              {
                return false;
              }
              //--- Note if bits == 15 then we have a tetrahedron as
              //--- our simplex (we could not reduce it in the sub-algorithm), since
              //--- the closest point must be orthogonal to all edges of the
              //--- simplex, this means origo must be inside the simplex i.e. we
              //--- have an intersection
            }
            while(m_bits<15 && !this->is_approx_zero(g));
            this->compute_points(m_bits, pa, pb);
            return true;
          }

          /**
          * Closest Point Queury.
          * This is the GJK version as Gino van den Bergen has presented it.
          *
          * @param A       A pointer to one convex volume
          * @param B       A pointer to another convex volume
          * @param pa      A pointer to a vector, which upon return contains a closest point on A to B
          * @param pb      A pointer to a vector, which upon return contains a closest point on B to A
          */
          template<typename convex_shape_type1, typename convex_shape_type2>
          void get_closest_points(
            convex_shape_type1 & A
            , convex_shape_type2 & B
            , vector3_type & pa
            , vector3_type & pb
            )
          {
            using std::max;
            using std::sqrt;

            vector3_type const zero = vector3_type( value_traits::zero(), value_traits::zero(), value_traits::zero() );

            vector3_type  vA = A.get_support_point(zero);
            vector3_type  vB = B.get_support_point(zero);
            vector3_type  v  = vA - vB;

            real_type dist = sqrt(v*v);

            m_bits     = 0;
            m_all_bits = 0;
            real_type mu = value_traits::zero();

            //--- Note if bits == 15 then we have a tetrahedron as
            //--- our simplex (we could not reduce it in the sub-algorithm), since
            //--- the closest point must be orthogonal to all edges of the
            //--- simplex, this means origo must be inside the simplex i.e. we
            //--- have an intersection
            while(m_bits<15 && dist > m_abs_error)
            {
              //--- Determine a position in y, which is free
              //--- and can be used for storing the new support
              //--- point in.
              m_last     = 0;
              m_last_bit = 1;

              while(m_bits & m_last_bit)//--- Loop until we find an entry not used in the current simplex
              {
                ++m_last;
                m_last_bit <<= 1;
              }

              m_p[m_last] = A.get_support_point( - v  );
              m_q[m_last] = B.get_support_point(   v  );

              vector3_type w = m_p[m_last] - m_q[m_last];

              mu = max( mu, ( dot(v,w) / dist) );

              if(dist - mu <= dist * m_rel_error)
              {
                break;
              }

              if(this->is_degenerate(w))
              {
                break;
              }

              m_y[m_last] = w;

              m_all_bits  = m_bits | m_last_bit;
              if(!this->get_closest(v))
              {
                break;
              }
              dist = sqrt( dot(v,v) );
            }
            this->compute_points(m_bits, pa, pb);
          }

        private:

          /**
          * Approximately Zero Test.
          *
          * @param vec  A pointer to a vector, which is should be tested
          *             if it is approximately zero.
          *
          * @return     If v is approximately zero (within the absolute
          *             error tolerance) then the return value is true
          *             otherwise it is false.
          */
          bool is_approx_zero(vector3_type & vec)
          {
            return (vec*vec < m_abs_error2);
          }

          /**
          * Compute Determinants
          * This method is responsible for updating the
          * determinant and dot product cache tables.
          */
          void compute_determinants()
          {
            //--- We are about to "add" y[last] to the current
            //--- simplex...

            //--- First we update the dot-product cache table
            for(int i=0, bit=1; i<4; ++i, bit <<=1)
            {
              //--- If y[i] is in current simplex then
              if(m_bits & bit)
              {
                m_dp[i][m_last] = m_dp[m_last][i] = m_y[i] * m_y[m_last];
              }
            }
            m_dp[m_last][m_last] = m_y[m_last] * m_y[m_last];
            //--- Afterwards we update the sub-determinants cache table
            //---
            //---  Notice that first index indicates the set of
            //---  indices $Is$

            //--- We start with base case of the recursive
            //--- computation, that is: $\Delta_i (\{ y_i\}) = 1$
            m_det[m_last_bit][m_last] = value_traits::one();
            //--- This is the case
            //---
            //---  $\Delta_{k}( X  \cup \{ y_k \} ) = \sum_{i\in I_x} \Delta_i(X) (y_i y_m - y_i y_k), m \in I_x, k \in I\I_x$
            //---
            //--- Where we know that the current simplex has exactly two indices
            for(int j=0, sj=1; j<4; ++j, sj <<= 1)
            {
              if(m_bits & sj)
              {
                int s2 = sj | m_last_bit;
                m_det[s2][j]      = m_dp[m_last][m_last] - m_dp[m_last][j];
                m_det[s2][m_last] = m_dp[j][j]           - m_dp[j][m_last];
                for(int k=0, sk=1; k<j; ++k, sk <<= 1)
                {
                  if(m_bits & sk)
                  {
                    int s3 = sk | s2;
                    m_det[s3][k]      = m_det[s2][j]                 * (m_dp[j][j] - m_dp[j][k]     ) + m_det[s2][m_last]              * (m_dp[m_last][j] - m_dp[m_last][k]);
                    m_det[s3][j]      = m_det[sk | m_last_bit][k]    * (m_dp[k][k] - m_dp[k][j]     ) + m_det[sk | m_last_bit][m_last] * (m_dp[m_last][k] - m_dp[m_last][j]);
                    m_det[s3][m_last] = m_det[sk|sj][k]              * (m_dp[k][k] - m_dp[k][m_last]) + m_det[sk | sj][j]              * (m_dp[j][k]      - m_dp[j][m_last]);
                  }
                }
              }
            }
            //--- all_bits = bits | last = 15 = 2^0 + 2^1 + 2^2+ 2^3  => current simplex has three points
            //---
            //--- $I_x$ contains three indices, if we know the last index, k, we can easily write up
            //---
            //---  $\Delta_{k}( X  \cup \{ y_k \} ) = \sum_{i\in I_x} \Delta_i(X) (y_i y_m - y_i y_k), m \in I_x, k \in I\I_x$
            if(m_all_bits == 15)
            {
              //--- k = 0 => bits must of $I_x$ be 14
              m_det[15][0] = m_det[14][1] * (m_dp[1][1] - m_dp[1][0]) + m_det[14][2] * (m_dp[2][1] - m_dp[2][0]) + m_det[14][3] * (m_dp[3][1] - m_dp[3][0]);
              //--- k = 0 => bits must of $I_x$ be 13
              m_det[15][1] = m_det[13][0] * (m_dp[0][0] - m_dp[0][1]) + m_det[13][2] * (m_dp[2][0] - m_dp[2][1]) + m_det[13][3] * (m_dp[3][0] - m_dp[3][1]);
              //--- k = 0 => bits of $I_x$ must be 11
              m_det[15][2] = m_det[11][0] * (m_dp[0][0] - m_dp[0][2]) + m_det[11][1] * (m_dp[1][0] - m_dp[1][2]) + m_det[11][3] * (m_dp[3][0] - m_dp[3][2]);
              //--- k = 0 => bits of $I_x$ must be 7
              m_det[15][3] = m_det[7][0]  * (m_dp[0][0] - m_dp[0][3]) + m_det[7][1]  * (m_dp[1][0] - m_dp[1][3]) + m_det[7][2]  * (m_dp[2][0] - m_dp[2][3]);
            }
          }

          /**
          * Validate Simplex.
          * This method validates a specified simplex.
          *
          * @param s     An index set $Is$ of the vertices in the
          *              simplex, representated as a bitmask.
          *
          * @return      If the simplex is valid then the return
          *              value is true otherwise it is false.
          */
          bool is_valid(int s)
          {
            for(int i=0, bit=1; i<4; ++i, bit <<= 1)
            {
              //--- Test whatever y[i] is either part of current
              //--- simplex or is the last support point
              if(m_all_bits & bit)
              {
                //--- Test if y[i] is part of the new better simplex
                if(s & bit)
                {
                  //--- Test if the new better simplex is valid
                  if(m_det[s][i]<=0)
                  {
                    return false;
                  }
                }
                else if(m_det[ (s|bit) ][i] > 0)
                {
                  //--- See if we have overlooked another possibility for a better new simplex???
                  return false;
                }
              }
            }
            //--- We can now conclude that the new better
            //--- simplex, specified by the index set $I_s$
            //--- was valid
            return true;
          }

          /**
          * Compute Vector.
          * This method is used to compute a vector to the point
          * on the current simplex (representated by the bitmask,
          * bits), which is closest to the origin. Assume we are
          * in the $k$'th iteration, then we have just computed
          * $W_{k+1}$ and now we need to compute $\vec v_{k+1}$.
          *
          * Note this method is basically just Cramer's Rule, which
          * uses the determinant that was computed previously.
          *
          * @param new_bits  The index set of the new simplex $W_{k+1}$
          *                  representated as a bitmask.
          * @param vec       A pointer to a vector which upon return
          *                  contains $\vec v_{k+1}$
          */
          void compute_vector(int new_bits, vector3_type & vec)
          {
            real_type sum = value_traits::zero();
            vec.clear();
            for(int i=0, bit=1; i<4; ++i, bit <<= 1)
            {
              //--- Test whatever y[i] is part of the current simplex
              if(new_bits & bit)
              {
                sum += m_det[new_bits][i];
                vec += m_y[i] * m_det[new_bits][i]; //--- Observe this is the denominator of $\lambda_i$
              }
            }
            vec *= value_traits::one()/sum;  //--- Observe "sum" is the numerator of all the $\lambda_i$'s
          }

          /**
          * Compute Points.
          * This method is used to compute the closest points. Assuming
          * we have found the final simplex.
          *
          * Note this method is basically just Cramer's Rule, which
          * uses the determinant that was computed previously.
          *
          * @param new_bits  The index set of the final simplex
          *                  representated as a bitmask.
          * @param p1        A pointer to a vector which upon return
          *                  contains the closest point on object A.
          * @param p2        A pointer to a vector which upon return
          *                  contains the closest point on object B.
          */
          void compute_points(int new_bits, vector3_type & p1, vector3_type & p2)
          {
            real_type sum = value_traits::zero();
            p1.clear();
            p2.clear();
            for (int i=0, bit=1; i<4; ++i, bit <<= 1)
            {
              if(new_bits & bit)
              {
                sum += m_det[new_bits][i];
                p1  += m_p[i]*m_det[new_bits][i]; //--- Observe this is the denominator of $\lambda_i$
                p2  += m_q[i]*m_det[new_bits][i]; //--- Observe this is the denominator of $\lambda_i$
              }
            }
            if(sum)
            {
              real_type s = value_traits::one() / sum; //--- Observe "sum" is the numerator of all the $\lambda_i$'s
              p1 *= s;
              p2 *= s;
            }
          }

          //--- \f[
          //---   \vec v_{k+1} = v(conv(W_k \cup \{\vec w_k\}))
          //--- \f]
          //---
          //--- and
          //---
          //--- \f[
          //---   W_{k+1} = \mbox{smallest} X \subseteq W \cup \{w\} \quad \mbox{where} \quad \vec v \in conv(X)
          //--- \f]

          /**
          * Get Closest Point.
          * This method is the sub-algorithm of GJK, it refines the
          * current simplex and determines the closest point to the
          * origin of the newly computed simplex.
          *
          * This is the part of the pseudo code, that is stated as follows
          *
          * \f[
          *    \vec v_{k+1} = v(conv(W_k \cup \{\vec w_k\}))
          * \f]
          *
          * and
          *
          * \f[
          *   W_{k+1} = \mbox{smallest} X \subseteq W \cup \{w\} \quad \mbox{where} \quad \vec v \in conv(X)
          * \f]
          *
          * Note this method alters the bitmask, bits, when it finds a new better simplex.
          *
          * @param vec  A pointer to a vector, which upon return contains the closest
          *             point on the new simplex to the origin.
          *
          * @return     If a closest point could be found then the return
          *             value is true otherwise it is false.
          */
          bool get_closest(vector3_type & vec)
          {
            //--- Compute Determinants
            this->compute_determinants();
            //--- Now perform an exhaustive search for the best set of
            //--- support points to form the new simplex
            for(int s=m_bits; s; --s)
            {
              //--- Test if the index set Is is a subset of the current
              //--- simplex
              if((s & m_bits) == s)
              {
                //--- Test if the subset and the new support
                //--- point forms a best/minimum simplex
                if( this->is_valid( s | m_last_bit) )
                {
                  //--- We have found a new best simplex
                  //--- First we remember its index set
                  m_bits = s | m_last_bit;
                  //--- Afterwards we compute the closest
                  //--- point to the origin, which will be
                  //--- our new separation axe for the next
                  //--- iteration
                  this->compute_vector(m_bits, vec);
                  return true;
                }
              }
            }
            //--- We could not find a best simplex with 2 or
            //--- 3 support points, maybe we can find a best
            //--- simplex with a single point? Which necessarily
            //--- must be the last found support point.
            if(this->is_valid(m_last_bit))
            {
              m_bits = m_last_bit;
              vec = m_y[m_last];
              return true;
            }
            return false;//--- Question when could this occur?
          }


          /**
          * Degeneracy Test Method.
          * This function is used for detecting degenerate cases
          * that cause termination problems due to rounding errors.
          *
          * @param vec   A pointer to the last computed support point.
          *
          * @return      If we have a degenerate case then the return
          *              value is true otherwise it is false.
          */
          bool is_degenerate(vector3_type & vec)
          {
            for(int i=0, bit=1; i<4; ++i, bit <<= 1)
            {
              if((m_all_bits & bit) && m_y[i]==vec)
                return true;
            }
            return false;
          }

        };

      } // namespace detail

      /**
      * Intersection Query.
      * This is the ISA-GJK version.
      * Note if coherence should be exploited one should surply the
      * method with the separation axe computed from the previous
      * iteration. The initial value could be any arbitary vector, we
      * sugest that you use the difference between the centers of A and B.
      *
      * @param A       A pointer to one convex volume
      * @param B       A pointer to another convex volume
      * @param g       A pointer to a vector, which contains an
      *                inital guess for a separation axe, between
      *                A and B. Upon return the vector will contain
      *                the separation axe, computed by GJK.
      *
      * @return        If A and B are intersecting then the return
      *                value is true otherwise it is false.
      */
      template< typename convex_shape_type1, typename convex_shape_type2, typename vector3_type >
      bool is_intersecting(convex_shape_type1 & A,convex_shape_type2 & B,vector3_type & g)
      {
        OpenTissue::gjk::obsolete::detail::GJK<vector3_type> algorithm;
        return algorithm.is_intersecting( A, B, g);
      }

      /**
      * Common Point Queury.
      * In case of intersection a common point between the two
      * specified convex volumes is computed.
      *
      * Note both pa and pb will contain the same value, this
      * is because we assume that A and B are represented in
      * the same coordinate frame. GJK could take affine
      * transformations of A and B into account. Meaning that
      * A and B could be given in their local frame, but we
      * have no need for this!!!
      *
      * @param A       A pointer to one convex volume
      * @param B       A pointer to another convex volume
      * @param g       A pointer to a vector, which contains an
      *                inital guess for a separation axe, between
      *                A and B. Upon return the vector will contain
      *                the separation axe, computed by GJK.
      * @param pa      A pointer to a vector, which upon return
      *                contains a closest point on A to B
      * @param pb      A pointer to a vector, which upon return
      *                contains a closest point on B to A
      *
      * @return        If A and B are intersecting then the return
      *                value is true otherwise it is false.
      */
      template< typename convex_shape_type1, typename convex_shape_type2, typename vector3_type>
      bool get_common_point(convex_shape_type1 & A, convex_shape_type2 & B, vector3_type & g, vector3_type & pa, vector3_type & pb)
      {
        OpenTissue::gjk::obsolete::detail::GJK<vector3_type> algorithm;
        return algorithm.get_common_point( A, B, g, pa, pb);
      }

      /**
      * Closest Point Queury.
      * This is the GJK version as Gino van den Bergen has presented it.
      *
      * @param A       A pointer to one convex volume
      * @param B       A pointer to another convex volume
      * @param pa      A pointer to a vector, which upon return contains a closest point on A to B
      * @param pb      A pointer to a vector, which upon return contains a closest point on B to A
      */
      template< typename convex_shape_type1, typename convex_shape_type2, typename vector3_type>
      void get_closest_points(convex_shape_type1 & A, convex_shape_type2 & B, vector3_type & pa, vector3_type & pb)
      {
        OpenTissue::gjk::obsolete::detail::GJK<vector3_type> algorithm;
        algorithm.get_closest_points( A, B, pa, pb);
      }

    } // namespace obsolete
  } // namespace gjk
} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_H
#endif
