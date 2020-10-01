#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_CONTACTS_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_CONTACTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/graphics/core/gl/gl_util.h>

namespace OpenTissue
{
  namespace mbd
  {

    template<typename configuration_type>
    void draw_contacts(configuration_type /*const*/ & configuration)
    {
      typedef typename configuration_type::body_type            body_type;
      typedef typename configuration_type::edge_type            edge_type;
      typedef typename edge_type::contact_iterator              contact_iterator;
      typedef typename body_type::real_type                     real_type;
      typedef typename body_type::vector3_type                  vector3_type;
      typedef typename body_type::matrix3x3_type                matrix3x3_type;
      typedef typename body_type::quaternion_type               quaternion_type;
      typedef typename body_type::value_traits                  value_traits;
      typedef typename configuration_type::edge_iterator        edge_iterator;

      for(edge_iterator edge = configuration.edge_begin();edge!=configuration.edge_end();++edge)
      {
        if(edge->is_up_to_date())
        {
          for(contact_iterator contact = edge->contact_begin();contact!=edge->contact_end();++contact)
          {
            gl::ColorPicker(value_traits::one(),value_traits::one(),value_traits::zero());
            gl::DrawPoint(contact->m_p,0.025);
            gl::ColorPicker(value_traits::zero(),value_traits::zero(),value_traits::one());
            gl::DrawVector(contact->m_p,contact->m_n*0.2,0.3,false);
            gl::ColorPicker(value_traits::one(),value_traits::zero(),value_traits::zero());
            if(contact->m_t.size()>0)
              gl::DrawVector(contact->m_p,contact->m_t[0]*0.2,0.3,false);
          }
        }
      }
    }


  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_CONTACTS_H
#endif 
