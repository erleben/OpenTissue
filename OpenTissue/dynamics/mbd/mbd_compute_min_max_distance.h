#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_MIN_MAX_DISTANCE_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_MIN_MAX_DISTANCE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/gl/gl_util.h>

#include <OpenTissue/core/math/math_constants.h>

#include <boost/cast.hpp>

namespace OpenTissue
{
  namespace mbd
  {
    template<typename configuration_type, typename real_type2>
    void compute_min_max_distance(configuration_type /*const*/ & configuration, real_type2 & min_value, real_type2 & max_value)
    {
      using std::max;
      using std::min;

      typedef typename configuration_type::body_type            body_type;
      typedef typename configuration_type::edge_type            edge_type;
      typedef typename edge_type::contact_iterator              contact_iterator;
      typedef typename body_type::real_type                     real_type;
      typedef typename body_type::vector3_type                  vector3_type;
      typedef typename body_type::matrix3x3_type                matrix3x3_type;
      typedef typename body_type::quaternion_type               quaternion_type;
      typedef typename configuration_type::edge_iterator    edge_iterator;

      real_type m = OpenTissue::math::detail::highest<real_type>();
      real_type M = OpenTissue::math::detail::lowest<real_type>();

      for(edge_iterator edge = configuration.edge_begin();edge!=configuration.edge_end();++edge)
      {
        if(edge->is_up_to_date())
        {
          for(contact_iterator contact = edge->contact_begin();contact!=edge->contact_end();++contact)
          {
            m=min(m,contact->m_distance);
            M=max(m,contact->m_distance);
          }
        }
      }

      min_value = boost::numeric_cast<real_type2>(m);
      max_value = boost::numeric_cast<real_type2>(M);
    }

  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_MIN_MAX_DISTANCE_H
#endif 
