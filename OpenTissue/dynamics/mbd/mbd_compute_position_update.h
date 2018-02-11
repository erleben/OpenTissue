#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_POSITION_UPDATE_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_POSITION_UPDATE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>
#include <OpenTissue/core/math/math_functions.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Position Update Method.
    * This method implements a modified position update, ie. 
    *
    *        $\vec s = \vec s + \Delta t \mat S \vec u$
    *
    * The modification lies in the ability to use finite rotatation
    * updates. This allows an end user to model high-speed spinning
    * objects to use finite rotation updates. Doing so will greatly
    * improve upon the accuracy.
    *
    * The drawback being that the finite update is much slower than the
    * infinite update.
    *
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param s       Must have 7*n entries is the current position - probably rcm + quaternion
    * @param u       Must have 6*n entries velocity - linear and angular
    * @param h       Timestep size
    * @param snew    Must be of same size as s - new position - we call with the same vector as s
    *
    */
    template<typename indirect_body_iterator,typename vector_type,typename real_type>
    void compute_position_update(
      indirect_body_iterator begin
      , indirect_body_iterator end
      , vector_type const & s
      , vector_type const & u
      , real_type const & h
      , vector_type & snew      
      )
    {
      typedef typename indirect_body_iterator::value_type     body_type;
      typedef typename body_type::math_policy                 math_policy;
      typedef typename body_type::value_traits                value_traits;
      typedef typename body_type::vector3_type                vector3_type;
      typedef typename body_type::quaternion_type             quaternion_type;
      typedef typename vector_type::size_type                 size_type;

      using std::cos;

      vector3_type W;
      vector3_type r_axis;
      vector3_type W_finite;
      vector3_type W_infinite;

      quaternion_type Q;
      quaternion_type Qtmp;
      quaternion_type Qdot;

      size_type n = std::distance(begin,end);      

      if(s.size()!=7*n)
        throw std::logic_error("compute_position_update(): s has incorrect dimension");
      if(u.size()!=6*n)
        throw std::logic_error("compute_position_update(): u has incorrect dimension");

      if(&s != &snew)
        math_policy::resize( snew, 7*n  );

      typename vector_type::const_iterator sval    = s.begin();
      typename vector_type::const_iterator uval    = u.begin();
      typename vector_type::iterator       snewval = snew.begin();

      real_type h_half = h / value_traits::two();

      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        if(body->is_active())
        {
          assert(is_number(*uval) || !"compute_position_update(): non number encountered");
          assert(is_number(*sval) || !"compute_position_update(): non number encountered");
          *snewval++ = *sval++ + (h * (*uval++));
          assert(is_number(*uval) || !"compute_position_update(): non number encountered");
          assert(is_number(*sval) || !"compute_position_update(): non number encountered");
          *snewval++ = *sval++ + (h * (*uval++));
          assert(is_number(*uval) || !"compute_position_update(): non number encountered");
          assert(is_number(*sval) || !"compute_position_update(): non number encountered");
          *snewval++ = *sval++ + (h * (*uval++));
          assert(is_number(*sval) || !"compute_position_update(): non number encountered");
          Q.s() = *sval++;
          assert(is_number(*sval) || !"compute_position_update(): non number encountered");
          Q.v()(0) = *sval++;
          assert(is_number(*sval) || !"compute_position_update(): non number encountered");
          Q.v()(1) = *sval++;
          assert(is_number(*sval) || !"compute_position_update(): non number encountered");
          Q.v()(2) = *sval++;
          assert(is_number(*uval) || !"compute_position_update(): non number encountered");
          W(0) = *uval++;
          assert(is_number(*uval) || !"compute_position_update(): non number encountered");
          W(1) = *uval++;
          assert(is_number(*uval) || !"compute_position_update(): non number encountered");
          W(2) = *uval++;
          if( body->has_finite_rotation_update())
          {
            if(body->has_finite_rotation_axis())
            {
              //--- Split angular velocity into two components, one
              //--- parallel to the axe (W_finite) and the other
              //--- orthogonal to the axe (W_infinite)
              body->get_finite_rotation_axis(r_axis);
              real_type tmp = r_axis * W;
              W_finite    = r_axis * tmp;
              W_infinite  = W - W_finite;

              //--- Apply finite rotation around axe
              real_type theta = tmp*h_half;
              real_type tmp2 = OpenTissue::math::sinc(theta)*h_half;
              Qtmp.s() = cos(theta);
              Qtmp.v() = W_finite;
              Qtmp.v() *= tmp2;
              Q = Qtmp % Q;

              //--- Now apply the remaing rotation as an infinitesimal update
              Q += (W_infinite % Q)* h_half;
            }
            else
            {
              //--- Act like we do a finite rotation of angle = |W|*h
              real_type theta = length(W)*h_half;
              real_type tmp2 = OpenTissue::math::sinc(theta)*h_half;
              Qtmp.s() = cos(theta);
              Qtmp.v() = W*tmp2;
              Q = Qtmp%Q;
            }
          }
          else
          {
            //--- Just do an infinitesimal rotation update (i.e. a forward euler step)
            Q += (W % Q)*h_half;
          }
          Q = normalize(Q);//--- To counter-act numerical problems
          *snewval++ = Q.s();
          *snewval++ = Q.v()(0);
          *snewval++ = Q.v()(1);
          *snewval++ = Q.v()(2);
        }
      }
    }

    template<typename group_type,typename vector_type,typename real_type>
    void compute_position_update(
      group_type const & group
      , vector_type const & s
      , vector_type const & u
      , real_type const & h
      , vector_type & snew      
      )
    {
      compute_position_update(group.body_begin(),group.body_end(),s,u,h,snew);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_POSITION_UPDATE_H
#endif
