#ifndef OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_NAIVE_BLEND_SCHEDULER_H
#define OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_NAIVE_BLEND_SCHEDULER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/animation/animation_interface.h>

#include <boost/iterator/indirect_iterator.hpp>

#include <list>
#include <algorithm>
#include <cassert>

namespace OpenTissue
{
  namespace animation
  {
    /**
    * A Blend Scheduler.
    * This class provides a naïve, simple, and primitive blend scheduler.
    *
    * It performs a straightforward simple flat blend of the added
    * animations in an incremental fashion.
    */
    template<typename skeleton_type_>
    class NaiveBlendScheduler
    {
    protected:

      typedef          skeleton_type_                       skeleton_type;
      typedef typename skeleton_type::bone_type             bone_type;
      typedef typename skeleton_type::math_types            math_types;
      typedef typename math_types::coordsys_type            coordsys_type;
      typedef typename math_types::real_type                real_type;
      typedef          Interface<skeleton_type>    animation_interface;

      typedef std::list<animation_interface*>                                        animation_ptr_container;
      typedef typename animation_ptr_container::iterator                             animation_ptr_iterator;
      typedef boost::indirect_iterator<animation_ptr_iterator,animation_interface>   animation_iterator;

    protected:

      animation_ptr_container    m_animations;   ///< A collection of animations that should be blended.

    public:

      NaiveBlendScheduler(){}

    public:

      void clear() {      m_animations.clear();    }

      void add(animation_interface * animation)
      {
        assert(animation || !"BlendScheduler::add(): Animation was null");
        assert(find(m_animations.begin(),m_animations.end(),animation) == m_animations.end() || !"BlendScheduler::add(): Animation was already added.");

        //--- TODO: 
        //---
        //--- Extend with parameters and do whatever is needed
        //--- to add the specified animation to the blend!!!

        m_animations.push_back(animation);
      }

      void remove(animation_interface * animation)
      {
        //--- TODO: 
        //---
        //--- Extend with parameters and do whatever is needed
        //--- to add the specified animation to the blend!!!

        m_animations.remove(animation);
      }

      /**
      * Compute Duration of Animation.
      *
      * @return    The duration of the longest animation in the blend.
      */
      real_type compute_duration()
      {
        //--- TODO:
        //---
        //--- Modify code, so the correct duration is returned, depending
        //--- on the choice of blend scheduling algorithm!

        real_type duration = 0;
        animation_iterator begin      = m_animations.begin();
        animation_iterator end        = m_animations.end();
        animation_iterator animation;
        for(animation=begin;animation!=end;++animation)
        {
          duration = std::max(duration,animation->compute_duration());
        }
        return duration;
      }

      /**
      * Compute Pose.
      * This method determines how to blend all animations into one resulting pose.
      *
      * @param skeleton
      * @param global_time
      */
      void compute_pose(skeleton_type & skeleton, real_type const & global_time)
      {
        //--- TODO: 
        //---
        //--- Modify code, so the correct pose is returned, depending on the
        //--- choice of blend scheduling algorithm!

        animation_iterator begin      = m_animations.begin();
        animation_iterator end        = m_animations.end();
        animation_iterator animation;

        skeleton.clear_pose();
        for(animation=begin;animation!=end;++animation)
        {
          real_type local_time = global_time; //--- TODO: add timewarping...
          //--- TODO: add alignment curve
          //--- TODO: set blend weights: animation->set_weight()
          animation->blend_pose(skeleton,local_time);
        }
        skeleton.compute_pose();
        //--- TODO: possible constraint matching...?
      }

    };

  } // namespace animation
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_NAIVE_BLEND_SCHEDULER_H
#endif
