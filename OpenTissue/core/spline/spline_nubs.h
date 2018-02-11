#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_NUBS_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_NUBS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace spline
  {
    /**
    * Non-Uniform B-spline.
    * This class is an implementation of nonunform B-splines.
    * Example of notation and terminology.
    *
    * Given the knot vector \f$U = {u_i}\f$ for \f$i \in [0..m]\f$ we use the symbols
    *
    * i   :    0     1   ....   n     n+1     ...    m
    * U   = [ u_0   u_1  ....   u_n  u_{n+1}  ...  u_{n+k} ]
    *
    * Here are some more detailed explanations
    *
    * k       : is the order of the spline
    * m       : is the index of the last entry in the knot vector ie. ( size(U)-1 )
    * m+1     : is the total number of knots
    * n = m-k : is the index of the last entry in the control point array
    * n+1     : is the total number of control points.
    *
    * @tparam U   The knot value container type.
    * @tparam P   The control point container type.
    */
    template<typename U, typename P>
    class NUBSpline
    {
    public:

      typedef U              knot_container;
      typedef P              point_container;

    protected:

      U      m_U;            ///< Knot container.
      P      m_P;            ///< Controlpoint container.
      size_t m_k;            ///< The order of the spline.

    public:

      NUBSpline()
        : m_k(0u)
      {}

      /**
      * Construct a NUBSpline with order, knot-container and controlpoints.
      */
      NUBSpline(
          size_t const order
        , U const & knots
        , P const & controls
        )
      {

        if(order<= 0u)
          throw std::invalid_argument("order must be greater than zero");

        if(knots.size() <= 0u)
          throw std::invalid_argument("knots was empty");

        if(controls.size() <= 0u)
          throw std::invalid_argument("controls was empty");

        if( (knots.size() - controls.size())  != order)
          throw std::invalid_argument("Incorrect specification, n = m - k, was not fulfilled! ");

        this->m_U     = knots;
        this->m_P     = controls;        
        this->m_k     = order;
      }

      virtual ~NUBSpline() {      }

    public:

      /**
      * Retrive Knot Vector.
      * This method should retrieve a reference to an array containing
      * all the knots of the spline.
      *
      * Each knot corresponds to a parametric value of a data point. The
      * values in the knot vector should be an increasing function.
      *
      * The received knot values should be treated as read-only, that
      * is don't expect the curve segments, the data points and/or the
      * control points will reflect any changes you make to the knot
      * values.
      *
      * For the exact consequence of modifing the knotvalues "outside" the
      * spline class you should refer to the specific details of the implemention
      * of the method in the spline class you use.
      *
      * @return    A reference to an array of knotvalues of the spline.
      */
      U       & get_knot_container()       {   return this->m_U;   }
      U const & get_knot_container() const {   return this->m_U;   }

      /**
      * Retrive ControlPoints.
      * This method should retrieve a reference to an array containing
      * all the control points of the spline.
      *
      * The received control points should be treated as read-only, that
      * is don't expect the curve segments and/or the data points will reflect any
      * changes you make to the received control points.
      *
      * For the exact consequences of modifing control points "outside" the
      * spline class you should refer to the specific details of the implemention
      * of the method in the spline class you use.
      *
      * @return    A reference to a container of vectors. Each vector represents
      *            a corresponding control point of the spline.
      */
      P       & get_control_container()       {        return this->m_P;      }
      P const & get_control_container() const {        return this->m_P;      }

      size_t const get_dimension()   const 
      {
        if(this->m_P.size() == 0u)
          return 0u;
        return this->m_P[0].size(); 
      }

      size_t const & get_order() const { return this->m_k;   }

    };

  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_NUBS_H
#endif
