#ifndef OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_KEYFRAME_JOINT_CHANNELS_H
#define OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_KEYFRAME_JOINT_CHANNELS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <vector>

namespace OpenTissue
{
  namespace animation
  {


    /**
    * A Keyframe Joint Transform.
    */
    template<typename skeleton_type_>
    class KeyframeJointChannels
    {
    public:

      typedef          skeleton_type_                     skeleton_type;
      typedef typename skeleton_type::math_types          math_types;
      typedef typename math_types::coordsys_type          coordsys_type;
      typedef typename math_types::real_type              real_type;

    protected:

      class Keyframe
      {
      public:

        real_type        m_time;   ///< Local time of this key tick.
        coordsys_type    m_value;  ///< The relative joint pose of this key tick.

      public:

        Keyframe(real_type const & local_time, coordsys_type const & value)
          : m_time(local_time)
          , m_value(value)
        {}
      };

      typedef std::vector<Keyframe>                   keyframe_container;
      typedef typename keyframe_container::iterator   keyframe_iterator;

    protected:

      size_t     m_bone_number;   ///< The number of the bone this joint transform affects.
      int                  m_next;          ///< Given time t this member holds the index to next keyframe, i.e. t <= key(m_next).time 
      int                  m_prev;          ///< Given time t this member holds the index to previous keyframe, i.e. t > key(m_next).time 
      keyframe_container   m_keyframes;     ///< Container of keyframes.

    public:

      KeyframeJointChannels() 
        : m_bone_number(~0)
        , m_next(-1)
        , m_prev(-1)
      {};

      virtual ~KeyframeJointChannels(){}

    public:

      size_t const & get_bone_number() const {return m_bone_number;}

      void set_bone_number(size_t const & number){ m_bone_number = number;}

      void add_key(real_type const & local_time, coordsys_type const & value)
      {
        //--- If first keyframe then simply add it
        if(m_keyframes.empty())
        {
          m_keyframes.push_back(Keyframe(local_time,value));
          return;
        }
        //--- If we know the new keyframe is later than any of the previous ones then just add it
        if(m_keyframes.back().m_time < local_time)
        {
          m_keyframes.push_back(Keyframe(local_time,value));
          return;
        }
        //--- We now know that the new keyframe is within the range of existing
        //--- keyframes, we must therefor initiate a search for the prober place
        //--- to insert the new keyframe.
        keyframe_iterator begin = m_keyframes.end();
        keyframe_iterator end   = m_keyframes.end();
        keyframe_iterator prev = begin;
        keyframe_iterator next = prev+1;
        //--- Do a linear search for the interval containing the new key (a
        //--- binary search would be faster)
        while( !((prev->m_time < local_time) && (local_time <= next->m_time)) )
        {
          ++prev;
          ++next;
        }
        assert(prev->m_time < local_time);
        assert(local_time <= next->m_time);
        //--- Test to see if we already have a keyframe at the same time as the new time.
        //--- We only allow one keyframe for each time, so we overwrite the old keyframe.
        if(next->m_time==local_time)
        {
          next->m_value = value;
          return;
        }
        //--- Finally we can safely add the new keyframe inbetween two old keyframes.
        m_keyframes.insert(next,Keyframe(local_time,value));
      }

    public:

      /**
      * Retrive Transform.
      * This method should return the coordinate transform coorespond to the specified time.
      *
      * @param local_time   The specified time.
      * @return             The coordinate transform.
      */
      coordsys_type get_value(real_type const & local_time)
      {
        int N = static_cast<int>( m_keyframes.size() ); // Avoid compiler warning

        //--- Handle case of no keyframes
        if(!N)
          return coordsys_type();

        //--- Handle case of out-of bounds query
        if(local_time <= m_keyframes[0].m_time)
          return m_keyframes[0].m_value;

        if(local_time >= m_keyframes[N-1].m_time)
          return m_keyframes[N-1].m_value;

        //--- Test if this is first query
        if(m_next==m_prev)
        {
          m_next = 1;
          m_prev = 0;
        }

        //--- Determine m_next and m_prev such that
        //---
        //---   m_keyframes[m_prev].m_time <  local_time <= m_keyframes[m_next].m_time
        //---
        while( m_next<N && local_time > m_keyframes[m_next].m_time )
          ++m_next;

        while( m_next>1 && local_time <= m_keyframes[m_next-1].m_time )
          --m_next;

        m_prev = m_next - 1;

        assert(m_prev>=0);
        assert(m_next>=1);
        assert(m_next<N);
        assert(m_prev<(N-1));

        //--- See whether we really need to interpolate between two keyframes
        if(m_keyframes[m_next].m_time == local_time)
          return m_keyframes[m_next].m_value;

        //--- We are now certain that we must interpolate a transform inbetween the previous
        //--- and next keyframes, so we start by computing the proper weight
        coordsys_type blended;

        real_type weight = (local_time - m_keyframes[m_prev].m_time) / (m_keyframes[m_next].m_time - m_keyframes[m_prev].m_time);
        assert(weight>=0);
        assert(weight<=1);

        //--- Finally we can interpolate the resulting keyframe
        blended.T() = m_keyframes[m_prev].m_value.T()*(1-weight) +  m_keyframes[m_next].m_value.T()*weight;
        blended.Q() = slerp(m_keyframes[m_prev].m_value.Q(),   m_keyframes[m_next].m_value.Q(),weight);

        return blended;
      };

      /**
      * Compute Duration.
      *
      * @return          Returns the duration in local time.
      */
      real_type  compute_duration()
      {
        if(m_keyframes.empty())
          return 0;
        return m_keyframes.back().m_time;
      };

    };

  } // namespace animation
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_ANIMATION_ANIMATION_KEYFRAME_JOINT_CHANNELS_H
#endif
