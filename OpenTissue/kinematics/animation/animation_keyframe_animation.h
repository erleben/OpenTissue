#ifndef OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_KEYFRAME_ANIMATION_H
#define OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_KEYFRAME_ANIMATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_precision.h> //--- Needed for math::working_precision
#include <OpenTissue/kinematics/animation/animation_interface.h>
#include <OpenTissue/kinematics/animation/animation_keyframe_joint_channels.h>

#include <list>
#include <cassert>


namespace OpenTissue
{
  namespace animation
  {

    /**
    * A Keyframe Animation.
    */
    template<typename skeleton_type_>
    class KeyframeAnimation : public Interface<skeleton_type_>	
    {
    public:

      typedef          skeleton_type_                       skeleton_type;
      typedef typename skeleton_type::bone_type             bone_type;
      typedef typename skeleton_type::math_types            math_types;
      typedef typename math_types::coordsys_type            coordsys_type;
      typedef typename math_types::real_type                real_type;
      typedef          Interface<skeleton_type>    animation_interface;
      typedef          KeyframeJointChannels<skeleton_type> channels_type;
      typedef std::list<channels_type>                      channels_container;
      typedef typename channels_container::iterator         channels_iterator;

    protected:

      channels_container            m_channels;   ///< All motion channels of all joints. Note that each channel_type is actually 6 motion channels for one specific joint/bone.

    public:

      KeyframeAnimation(){}

      /**
      *
      * @return        A pointer to the newly created keyframe transform.
      */
      channels_type * create_joint_channels()
      {
        m_channels.push_back(channels_type());
        return &m_channels.back();
      }

    public:

      void clear() {  m_channels.clear(); };

      /**
      * Blend Pose.
      *
      * @param skeleton
      * @param local_time
      */
      void blend_pose(skeleton_type & skeleton,real_type const & local_time)
      {
        typedef typename skeleton_type::bone_traits    bone_traits;

        static real_type const tiny = OpenTissue::math::working_precision<real_type>();

        if( this->m_weight < tiny )
          return;

        channels_iterator begin   = m_channels.begin();
        channels_iterator end     = m_channels.end();
        channels_iterator channel;

        for(channel=begin; channel!=end; ++channel)
        {
          coordsys_type value = channel->get_value(local_time);

          bone_type * bone = skeleton.get_bone(channel->get_bone_number());

          bone->accumulated_weight() += this->m_weight;
          real_type u = this->m_weight / bone->accumulated_weight();

          coordsys_type relative = bone_traits::convert( bone->relative() );

          relative.T() += u * (value.T() - relative.T());
          relative.Q() = slerp(relative.Q(),value.Q(),u);
          
          bone->relative() = bone_traits::convert( relative );
        }
      }

      /**
      * Compute Duration of Keyframe Animation.
      *
      * @return   The duration of the keyframe animation.
      */
      real_type compute_duration()
      {
        using std::max;
        real_type duration = real_type();  //--- by standard default constructed integral types are zero.

        channels_iterator begin   = m_channels.begin();
        channels_iterator end     = m_channels.end();
        channels_iterator channel;

        for(channel=begin; channel!=end; ++channel)
        {
          duration = max(duration,channel->compute_duration());
        }
        return duration;
      }

    };

  } // namespace animation
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_KEYFRAME_ANIMATION_H
#endif
