#ifndef OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_INTERFACE_H
#define OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_INTERFACE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace animation
  {


    /**
    * An Animation base Class.
    */
    template<typename skeleton_type_>
    class Interface
    {
    public:

      typedef          skeleton_type_               skeleton_type;
      typedef typename skeleton_type::math_types    math_types;
      typedef typename math_types::coordsys_type    coordsys_type;
      typedef typename math_types::real_type        real_type;

    protected:

      real_type                  m_weight;    ///< End user assigned weight, could be anything!!!

    public:

      Interface()
        : m_weight(1.0)
      {}

      virtual ~Interface(){}

    public:

      virtual void clear()
      {
        m_weight = 1.0;
      }


    public:

      real_type const & get_weight() const {  return m_weight; };

      void set_weight(real_type const & value) { assert(value>=0); m_weight= value; };

    public:

      /**
      * Blend Pose.
      *
      * @param skeleton
      * @param local_time
      */
      virtual void blend_pose(skeleton_type & skeleton, real_type const & local_time)=0;


      /**
      * Compute Duration.
      *
      * @return    The duration of the animation in local time.
      */
      virtual real_type compute_duration(void)=0;

    };

  } // namespace animation
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_INTERFACE_H
#endif
