#ifndef OPENTISSUE_COLLISION_GJK_GJK_SIMPLEX_H
#define OPENTISSUE_COLLISION_GJK_GJK_SIMPLEX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <stdexcept>

#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {

    /**
    * Simplex Type.
    * This class defines a minimal set of data that defines a
    * simplex. The class is specifically tailored to be used in
    * the GJK algorithm. That means it is limited completely to
    * a singleton simplex in a three dimensional space.
    *
    * Further the class contains storage for barycentric coordinates
    * and the original support points used to generate the simplex vertices.
    *
    * Notice that the barycentric coordinates are really only needed when
    * one wants to re-construct the two closest points between two convex
    * sets. This is explained in detail below. In the general distance between
    * point and convex set algorithm there is no need for these coordinates.
    * However, for our specific purpose we need to store them.
    *
    * The simplex vertices are obtained from a support function for the
    * Minikowsky difference between two convex point sets \f$A\f$ and
    * \f$B\f$.
    *
    * The Minikowsky Difference is defined as
    *
    * \f[
    *   A \ominus B = \{a-b |  a \in A, b \in B \}
    * \f]
    *
    * One can show that the support function, \f$S_{A \ominus B}\f$, of
    * the Minikowsky Difference can be evaluated as
    *
    * \f[
    *  S_{A \ominus B}( v ) = S_{A}( v ) - S_{B}( -v )
    * \f]
    *
    * This is used in the GJK algorithm to generate the simplex vertices. Thus
    * for every simplex vertex, \f$v_i\f$, there exist a corresponding support point, \f$a_i\f$, for \f$A\f$ and
    * one, \f$b_i\f$, for \f$B\f$. Thus we have
    *
    * \f[
    *   v_i = a_i - b_i
    * \f]
    *
    * Having found the cloest point, \f$q\f$, on the simplex to the origin. Then by the
    * very nature of the simplex we can write \f$q\f$ as a convex linear combination. That
    * is
    *
    * \f[
    *   q = w_1 v_1 + \cdots + w_n v_n
    * \f]
    *
    * where the \f$v_i\f$'s are the simplex vertices and \f$w_i\f$'s are scalar
    * weights such that
    *
    * \f[
    *   w_1 + \cdots + w_n = 1
    * \f]
    *
    * and \f$0 \leq w_i \leq 1\f$. Thus one immediately recognize the scalar weights
    * as the bary-centric coordinates of q.
    *
    * It is rather straightforward to compute the weights. Once we know them we can
    * use these to obtain the closest points between the two convex sets \f$A\f$ and \f$B\f$. This
    * is done as follows:
    *
    * \f[
    *   q = w_1 v_1 + \cdots + w_n v_n = w_1 (a_1 - b_1) + \cdots + w_n (a_n - b_n)
    * \f]
    *
    * Next re-order
    *
    * \f[
    *   q = \underbrace{ (w_1 a_1 + \cdots + w_n a_n ) }_{q_a} - \underbrace{ (w_1 b_1 + \cdots + w_n b_n ) }_{q_b}
    * \f]
    *
    * By convexity of \f$A\f$ and \f$B\f$ we know
    *
    * \f[ q_a \in A \qquad \text{and} \qquad q_b \in B \f}
    *
    * Thus we have found to members of \f$A\f$ and \f$B\f$ that gives the
    * minimum norm point in the Minikowsky difference. In other words we
    * have found two closest points between \f$A\f$ and \f$B\f$.
    *
    */
    template<typename vector3_type>
    class Simplex
    {
    public:

      typedef typename vector3_type::value_traits  value_traits;
      typedef typename vector3_type::value_type    real_type;

    public:

      int m_bitmask;    ///< Bit mask that identifies currently used
                        ///< entries. If (0x0001 & m_bitmask)==1 then
                        ///< it means that the first entry is used. If
                        ///< (0x0002 & m_bitmask)==1 the second is used
                        ///< and if (0x0004 & m_bitmask)==1 then the
                        ///< third one is used and if (0x0008 & m_bitmask)==1 the
                        ///< fourth entry is used.  In general if \f$(2^i & m_bitmask) == 1\f$
                        ///< then the i'th array entry is used.
      vector3_type m_v[4];    ///< The simplex vertices.
      real_type    m_w[4];    ///< Barycentric coordinates for the closest point on the simplex wrt. the simplex vertices.
      vector3_type m_a[4];    ///< The support points from object A corresponding to the simplex vertices.
      vector3_type m_b[4];    ///< The support points from object B corresponding to the simplex vertices.

    public:

      Simplex()
        : m_bitmask(0)
      {
        m_v[0].clear();
        m_v[1].clear();
        m_v[2].clear();
        m_v[3].clear();

        m_a[0].clear();
        m_a[1].clear();
        m_a[2].clear();
        m_a[3].clear();

        m_b[0].clear();
        m_b[1].clear();
        m_b[2].clear();
        m_b[3].clear();

        m_w[0] = value_traits::zero();
        m_w[1] = value_traits::zero();
        m_w[2] = value_traits::zero();
        m_w[3] = value_traits::zero();
      }

    };

    /**
    * Test if point is in Simplex.
    *
    *
    * @param p         The point that should be tested
    *                  for containment. It is assumed that
    *                  the point is a support point from a
    *                  support function.
    *
    * @param S         The simplex to test against.
    *
    * @return          If p is in the specified simplex then
    *                  the return value is true otherwise it
    *                  is false.
    */
    template<typename V>
    inline bool is_point_in_simplex( V const & p, Simplex<V> const & S)
    {
      // Note that p would be a support point of a convex set.
      //
      // We will work under the two assumptions:
      //
      // Assumption 1: Allthough a support function need not be onto we will assume
      // that if a support function will return the same support point
      // given the same search direction.
      //
      // Assumption 2:  Also we will assume that a support point is allways a corner
      // point of the convex hull of the convex set.
      //
      // Thus we know that any simplex would consist of support points that
      // are corners of the convex hull. Thus if suffice to check if p is one
      // of vertices of the simplex.
      //
      // Let us assume that p lies on some point on the boundary of the
      // simplex. Assume further that p is not the same as one of the
      // simplex vertices.
      //
      // This would imply that p is lying inside a face of the simplex (remember
      // a face of a simplex is simply the set spanned by a subset of the
      // simplex vertices). 
      //
      // Let v_i, v_j, and v_k be any three vertices of the face containing p.
      // An edge would have just two vertices and we leave this simpler case
      // to be proven by the reader. Thus continuing with our face we know by
      // the definition of a simplex that we can write
      //
      //   p = a_i v_i + a_j v_j + a_k v_k
      //
      // where 0 <= a_i, a_j, a_k <= 1 and a_i + a_j + a_k = 1
      //
      // However since v_i, v_j and v_k by definition are corners
      // of the convex hull. We know that all points of the form
      //
      //   q = b_i v_i + b_j v_j + b_k v_k
      //
      // where 0 <= b_i, b_j, b_k <= 1 and b_i + b_j + b_k = 1 lies
      // on the boundary of the convex hull. This contradicts the
      // assumption that p is a corner vertex of the convex hull.
      //
      // Thus the only way for p to be a vertex of the convex hull
      // is that if p is the same as one of the simplex vertices.
      //
      int used_bit = 1;
      for(size_t i = 0u; i < 4u; ++i)
      {
        if( (S.m_bitmask & used_bit) && (p == S.m_v[i]) )
          return true;
        used_bit <<= 1;
      }
      return false;
    }

    /**
    * Add Point to Simplex.
    *
    * @param p        The point to be added to the simplex. It
    *                 is assumed implicitly that this point is
    *                 linear independent of the points already
    *                 contained in the simplex.
    *
    * @param p_a      A corresponding point from set A.
    * @param p_b      A corresponding point from set B.
    *
    * @param S        The simplex where the point should be added.
    *
    */
    template<typename V>
    inline void add_point_to_simplex( V const & p, V const & p_a, V const & p_b , Simplex<V> & S)
    {
      typedef typename V::value_traits   value_traits;

      size_t free_idx  = 0u;
      int free_bit = 1;        

      // Loop until we find a free bit, that is a bit in the simplex bitmask that is set low.
      while(S.m_bitmask & free_bit)
      {
        ++free_idx;
        free_bit <<= 1;
      }

      if( free_idx >= 4u )
        throw std::logic_error("Simplex did not have any empty entries");

      S.m_a[free_idx] = p_a;
      S.m_b[free_idx] = p_b;
      S.m_v[free_idx] = p;
      S.m_w[free_idx] = value_traits::zero();

      // Update bitmask to reflect that the free bit is no longer free
      S.m_bitmask |= free_bit;       
    }

    /**
    * The Dimension of the Simplex.
    *
    * @param S   The simplex.
    *
    * @return    The dimension of the simplex, this is defined
    *            as the cardinality of the simplex vertex set.
    */
    template<typename V>
    inline size_t dimension(Simplex<V> const & S) 
    {  
      size_t size = 0u;

      if( S.m_bitmask & 1 )
        ++size;
      if( S.m_bitmask & 2 )
        ++size;
      if( S.m_bitmask & 4 )
        ++size;
      if( S.m_bitmask & 8 )
        ++size;
      return size;
    }

    /**
    * Test If Simplex is Full.
    * A full simplex is one where all vertices are in use. In
    * our case it means that we got a tetrahedron with 4 vertices.
    *
    * @param S   The simplex.
    *
    * @return    If the simplex got four vertices then the return
    *            value is true otherwise it is false.
    */
    template<typename V>
    inline bool is_full_simplex(Simplex<V> const & S) 
    {  
      return (S.m_bitmask == 15u);
    }

    /**
     * Get First Used Index in Simplex.
     * This function will determine the entry of the first used simplex vertex.
     *
     * @param bitmask    A simplex bitmask indicating usage of simplex vertices.
     * @param idx_A      Upon return this argument holds the entry.
     * @param bit_A      Upon return this argument holds the bit-flag of the first used entry.
     */
    inline void get_used_indices( int const & bitmask, size_t & idx_A, int & bit_A)
    {
     if(bitmask == 0u) 
       throw std::invalid_argument("no bits was set high in bitmask");

     size_t idx = 0u;

      bit_A = 1;
      for(; idx < 4; ++idx)
      {
        if( bitmask & bit_A )
          break;
        bit_A <<= 1;
      }
      idx_A = idx;

      assert( bit_A==1 || bit_A==2 || bit_A==4 || bit_A==8 || !"get_used_indices(): bit A was illegal");
      assert( idx_A >= 0u            || !"get_used_indices(): illegal index A value");
      assert( idx_A < 4u             || !"get_used_indices(): illegal index A value");
      assert( ((1 << idx_A) & bit_A) || !"get_used_indices(): mismatch between idx A and bit A");
    }

    /**
     * Get First Two Used Indices in Simplex.
     * This function will determine the entry of the first two used simplex vertices.
     *
     * @param bitmask    A simplex bitmask indicating usage of simplex vertices.
     * @param idx_A      Upon return this argument holds the first entry.
     * @param bit_A      Upon return this argument holds the bit-flag of the first used entry.
     * @param idx_B      Upon return this argument holds the second entry.
     * @param bit_B      Upon return this argument holds the bit-flag of the second used entry.
     */
    inline void get_used_indices(
      int const & bitmask
      , size_t & idx_A
      , int & bit_A
      , size_t & idx_B
      , int & bit_B
      )
    {
     if(bitmask == 0u) 
       throw std::invalid_argument("no bits was set high in bitmask");

      size_t idx = 0u;

      bit_A = 1;
      for(; idx < 4u; ++idx)
      {
        if( bitmask & bit_A )
          break;
        bit_A <<= 1;
      }
      idx_A = idx++;

      bit_B = bit_A << 1;
      for(; idx < 4u; ++idx)
      {
        if( bitmask & bit_B )
          break;
        bit_B <<= 1;
      }
      idx_B = idx;

      if(idx_B==4u)
        throw std::logic_error("simplex only had one used entry");

      assert( bit_A==1 || bit_A==2 || bit_A==4 || bit_A==8 || !"get_used_indices(): bit A was illegal");
      assert( idx_A >= 0u            || !"get_used_indices(): illegal index A value");
      assert( idx_A < 4u             || !"get_used_indices(): illegal index A value");
      assert( ((1 << idx_A) & bit_A) || !"get_used_indices(): mismatch between idx A and bit A");

      assert( bit_B==1 || bit_B==2 || bit_B==4 || bit_B==8 || !"get_used_indices(): bit B was illegal");
      assert( idx_B >= 0u            || !"get_used_indices(): illegal index B value");
      assert( idx_B < 4u             || !"get_used_indices(): illegal index B value");
      assert( ((1 << idx_B) & bit_B) || !"get_used_indices(): mismatch between idx B and bit B");
      
      assert( ((bit_A & bit_B) == 0) || !"get_used_indices(): Bit A and Bit B was the same");
      assert(  idx_A < idx_B         || !"get_used_indices(): idx A should be smaller than idx B");
    }

    /**
     * Get First Three Used Indices in Simplex.
     * This function will determine the entry of the first three used simplex vertices.
     *
     * @param bitmask    A simplex bitmask indicating usage of simplex vertices.
     * @param idx_A      Upon return this argument holds the first entry.
     * @param bit_A      Upon return this argument holds the bit-flag of the first used entry.
     * @param idx_B      Upon return this argument holds second the entry.
     * @param bit_B      Upon return this argument holds the bit-flag of the second used entry.
     * @param idx_C      Upon return this argument holds the third entry.
     * @param bit_C      Upon return this argument holds the bit-flag of the third used entry.
     */
    inline void get_used_indices( 
      int const & bitmask
      , size_t & idx_A
      , int & bit_A
      , size_t & idx_B
      , int & bit_B
      , size_t & idx_C
      , int & bit_C
      )
    {
     if(bitmask == 0u) 
       throw std::invalid_argument("no bits was set high in bitmask");

     size_t idx = 0u;

      bit_A = 1;
      for(; idx < 4; ++idx)
      {
        if( bitmask & bit_A )
          break;
        bit_A <<= 1;
      }
      idx_A = idx++;

      bit_B = bit_A << 1;
      for(; idx < 4; ++idx)
      {
        if( bitmask & bit_B )
          break;
        bit_B <<= 1;
      }
      idx_B = idx++;

      if(idx_B==4u)
        throw std::logic_error("simplex only had one used entry");

      bit_C = bit_B << 1;
      for(; idx < 4; ++idx)
      {
        if( bitmask & bit_C )
          break;
        bit_C <<= 1;
      }
      idx_C = idx;

      if(idx_C==4u)
        throw std::logic_error("simplex only had two used entries");


      assert( bit_A==1 || bit_A==2 || bit_A==4 || bit_A==8 || !"get_used_indices(): bit A was illegal");
      assert( idx_A >= 0u            || !"get_used_indices(): illegal index A value");
      assert( idx_A < 4u             || !"get_used_indices(): illegal index A value");
      assert( ((1 << idx_A) & bit_A) || !"get_used_indices(): mismatch between idx A and bit A");

      assert( bit_B==1 || bit_B==2 || bit_B==4 || bit_B==8 || !"get_used_indices(): bit B was illegal");
      assert( idx_B >= 0u            || !"get_used_indices(): illegal index B value");
      assert( idx_B < 4u             || !"get_used_indices(): illegal index B value");
      assert( ((1 << idx_B) & bit_B) || !"get_used_indices(): mismatch between idx B and bit B");
      
      assert( ((bit_A & bit_B) == 0) || !"get_used_indices(): Bit A and Bit B was the same");
      assert(  idx_A < idx_B         || !"get_used_indices(): idx A should be smaller than idx B");

      assert( bit_C==1 || bit_C==2 || bit_C==4 || bit_C==8 || !"get_used_indices(): bit B was illegal");
      assert( idx_C >= 0u            || !"get_used_indices(): illegal index B value");
      assert( idx_C < 4u             || !"get_used_indices(): illegal index B value");
      assert( ((1 << idx_C) & bit_C) || !"get_used_indices(): mismatch between idx B and bit B");
      
      assert( ((bit_A & bit_C) == 0) || !"get_used_indices(): Bit A and Bit C was the same");
      assert( ((bit_B & bit_C) == 0) || !"get_used_indices(): Bit B and Bit C was the same");
      assert(  idx_B < idx_C         || !"get_used_indices(): idx B should be smaller than idx C");

    }

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SIMPLEX_H
#endif
