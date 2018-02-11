#ifndef OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CONE_H
#define OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CONE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {
    /**
    * A Cone.
    * A Cone is placed with its apex on positive z-axis and
    * the base orthogonal to the negative z-axis.
    * 
    * The apex and base are placed equidistant wrt. x-y plane and
    * the distance is given by the half-height of the cone.
    *
    * The base is a circular disk of a specified radius. From the radius
    * and height of the cone one can compute the cone angle at the apex.
    * This angle is denoted alpha.
    */
    template<typename math_types>
    class Cone
    {
    protected:

      typedef typename math_types::real_type     T;
      typedef typename math_types::vector3_type  V;
      typedef typename math_types::value_traits  value_traits;

      T m_half_height;   ///< The half height of the cone. Default value is one.
      T m_base_radius;   ///< The radius of the circular base of the cone. Default value is one.

    public:

      /**
      * Get Cone Half height.
      *
      * @return    A reference to the cone half height.
      */
      T const & half_height() const { this->m_half_height; }
      T       & half_height()       { this->m_half_height; }

      /**
      * Get Cone Base Radius.
      *
      * @return    A reference to the cone base radius.
      */
      T const & base_radius() const { this->m_base_radius; }
      T       & base_radius()       { this->m_base_radius; }

    public:

      Cone()
        : m_half_height( value_traits::one() )
        , m_base_radius( value_traits::one() )
      {}

    public:

      V operator()(V const & v) const
      {
        using std::sqrt;

        assert( this->m_half_height >= value_traits::zero() || !"Cone::operator(): Negative half height");
        assert( this->m_base_radius >= value_traits::zero() || !"Cone::operator(): Negative base radius");

        T vv = dot(v,v);
        assert( is_number(vv) || !"Cone::operator(): NaN encountered");

        // Test if we have a valid search direction. If not we simply return some boundary point of the cone
        if (vv <= value_traits::zero() )
        {
          return V( value_traits::zero(), value_traits::zero(), -this->m_half_height );
        }
        
        // Now we know that we have a valid search direction. We proceed by
        // performing a case-by-case analysis. The theoretical back-ground
        // of the analysis is outlined below.

        // Since a Cone is radially symmetric around the z-axis we choose to
        // perform the analysis in the radial half-plane orthogonal to the
        // search direction.
        //
        // In this radial half plane the cone is simply a triangle. Looking like this
        //
        //                                  //
        //  z=+h --+        apex            //
        //         |\                       //
        //         | \                      //
        //  z=0 ---+  \                     //
        //         |   \                    //
        //         |    \                   //
        //  z=-h --+-----+---->             //
        //         |     |
        //        r=0   r=R
        //          base
        //
        // The height can be computes as
        //
        //   H = 2*h
        //
        // The angle, alpha at the apex, can be found from the
        // trigonometric relation
        //
        //   sin(alpha) = \frac{R}{H}
        //
        // If the appex is the support point then it must mean
        // that the search direction, s, is contained in the voronoi
        // region of the appex. By geometric relations we observe
        // that the voroni region is defined by the alpha-angle.
        //
        //                                _                             //
        //         |    /                 /|  search direction          //
        //         |  /                  /                              //
        //         |/  alpha            /                               //
        //  apex   +----------         / gamma                          //
        //         |\                 +-------                          //
        //         | \                                                  //
        //  z=0 ---+  \                                                 //
        //         |   \                                                //
        //         |    \                                               //
        //         +-----+---->                                         //
        //
        // So we need to test if angle, gamma, of the search direction is
        // larger than the apex angle. That is
        //
        //   gamma >= alpha   (*1)
        //
        // From geometry we find
        //                
        // sine(gamma) =  \frac{s_z}{\norm{s}}
        //
        // Fortunately the sine is a monotome function of the
        // interval [0..pi/2] so (*1) can stated as
        //
        //    sine(gamma) >= sine(alpha)
        //
        // By substitution of trigonometric terms this can be rewritten as
        //
        //   s_z >= \norm{s} \frac{B}{H}
        //
        // Observe that we have included the voronoi region of the inclined
        // cone side into the voronoi region of the appex. Therefor the next
        // voroni-region to consider is the base corner.
        //
        // If the first test fails then it suffices to test if the search
        // direction has any component in the radial direction. If this is
        // the case then the base corner must be a support point.
        //
        // If the two previous test-case both fails it means we have
        // a downward search direction and any point on the base of the
        // cone is a support point.

        T const sine_alpha = this->m_base_radius / (value_traits::two()*this->m_half_height);

        assert( is_number( sine_alpha ) || !"Cone::operator(): NaN encountered");
        assert( sine_alpha >= value_traits::zero() || !"Cone::operator(): sine(alpha) can not be negative");
        assert( sine_alpha <= value_traits::one() || !"Cone::operator(): sine(alpha) can not be larger than one");

        T const norm_v     = sqrt(vv);
        assert( is_number( norm_v ) || !"Cone::operator(): NaN encountered");
        assert( norm_v >= value_traits::zero() || !"Cone::operator(): Norm can not be negative");

        // Test if search direction is in Apex voronoi region
        if( v(2) >= norm_v*sine_alpha)
        {
          V const s = V( value_traits::zero(), value_traits::zero(), this->m_half_height );
          assert( is_number( s(0) ) || !"Cone::operator(): NaN encountered");
          assert( is_number( s(1) ) || !"Cone::operator(): NaN encountered");
          assert( is_number( s(2) ) || !"Cone::operator(): NaN encountered");
          return s;
        }

        T const norm_sigma     = sqrt( v(0)*v(0) + v(1)*v(1) );

        assert( is_number( norm_sigma ) || !"Cone::operator(): NaN encountered");
        assert( norm_sigma >= value_traits::zero() || !"Cone::operator(): Norm can not be negative");

        // Test if search direction has any radial component
        if(norm_sigma > value_traits::zero() )
        {
          V const s = V( this->m_base_radius*v(0)/norm_sigma, this->m_base_radius*v(1)/norm_sigma, -(this->m_half_height) );
          assert( is_number( s(0) ) || !"Cone::operator(): NaN encountered");
          assert( is_number( s(1) ) || !"Cone::operator(): NaN encountered");
          assert( is_number( s(2) ) || !"Cone::operator(): NaN encountered");
          return s;
        }

        // Search direction must be straight down        
        return V( value_traits::zero(), value_traits::zero(), -this->m_half_height );
      }

    };

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CONE_H
#endif
