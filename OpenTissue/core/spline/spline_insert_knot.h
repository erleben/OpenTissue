#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_INSERT_KNOT
#define OPENTISSUE_CORE_SPLINE_SPLINE_INSERT_KNOT
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/spline/spline_compute_knot_span.h>

namespace OpenTissue
{
  namespace spline
  {
    namespace detail
    {
      /**
      * Insert Knot.
      * This implementation is based on the algorithm in "Fundamentals of
      * Computer Aided Geometric Design", by J. Horschek and D. Lasser,
      * pp. 190-192.
      *
      * Note that the algorithm in the book does not treat the cases
      * where
      *
      *    r < K or   r > n-1
      *
      * For an open spline this will cause interpolation of non-existent
      * de Boor points and therefor cause array out bound exceptions!!! In
      * short we must require the multiplicity at the endpoints to be K!!!
      *
      * @param u    The new knot-value that should be inserted.
      * @param k    Order.
      * @param U    Knot-container that upon return will contain the new knot u.
      * @param P    Controlpoint container that upon return will contain the original
      *             and new de Boor points.
      */
      template<typename knot_container, typename point_container>
      inline void insert_knot( 
        typename knot_container::value_type const & u
        , int k
        , knot_container & U
        , point_container & P
        )
      {
        typedef typename point_container::value_type V;
        typedef typename knot_container::value_type  T;

        if(k<=0)
          throw std::invalid_argument("the spline order must be one or more");

        int i;
        int const m   = U.size()-1;
        int const n   = m-k;
        int const dim = P[0].size();

        knot_container  U_new(n+k+2);
        point_container P_new(n+2);

        for(i = 0; i <(n+2); ++i )
        {
          P_new[i].resize(dim);
        }

        //--- Determine the interval of knot values, where
        //--- the new knot value belongs to.
        int r = detail::compute_knot_span(u, k, U);

        //--- Insert the new knot.
        for(i=0; i<(n+k+2); ++i)
        {// i < |T'|
          if(i<=r)
            U_new[i] = U[i];
          else if(i>(r+1))
            U_new[i] = U[i-1];
          else
            U_new[r+1] = u;
        }

        //--- Compute the new Control Points
        V tmpA(dim);
        V tmpB(dim);

        for(i=0;i<(n+2);++i)
        {// i < |P'|

          P_new[i].clear();

          if(i<=(r-k+1))
          {
            P_new[i] = P[i];
          }
          else if(i>r){
            P_new[i] = P[i-1];
          }
          else
          {
            //---               u - u_i
            //---  a_i = --------------------
            //---          u_{i+k-1} - u_i
            T ai = (u - U[i])/(U[i+k-1]-U[i]);

            tmpA = P[i-1]*(1-ai);
            tmpB = P[i] * ai;
            P_new[i] = tmpA;
            P_new[i] += tmpB;
          }
        }

        U = U_new;
        P = P_new;
      }

    } // namespace detail


    /**
     * Insert Knot into Spline.
     * This function will create a new knot value in the
     * spline and a corresponding control point. Thew new
     * data is created such that the spline shape is unaffected
     * by the new knot-value.
     *
     * @param u       The parameter value at which the new knot value should be created.
     * @param spline  The spline where the new knot value should be inserted.
     */
    template<typename spline_type>
    inline void insert_knot(  typename spline_type::knot_container::value_type const & u, spline_type & spline )
    {
      detail::insert_knot( u, spline.get_order(), spline.get_knot_container(), spline.get_control_container() );
    }


  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_INSERT_KNOT
#endif
