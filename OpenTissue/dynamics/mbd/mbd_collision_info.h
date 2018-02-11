#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_COLLISION_INFO_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_COLLISION_INFO_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp>  // needed for boost::numeric_cast

namespace OpenTissue
{

  namespace mbd
  {

    template<typename mbd_types>
    class CollisionInfo
    {
    public:

      typedef typename mbd_types::math_policy        math_policy;

    protected:

      typedef typename math_policy::real_type        real_type;
      typedef typename math_policy::value_traits     value_traits;
      typedef typename math_policy::vector3_type     vector3_type;
      typedef typename math_policy::coordsys_type    coordsys_type;
      typedef typename math_policy::quaternion_type  quaternion_type;
      typedef typename math_policy::matrix3x3_type   matrix3x3_type;

    public:

      typedef typename mbd_types::geometry_type                geometry_type;
      typedef typename mbd_types::body_type                    body_type;
      typedef typename mbd_types::material_type                material_type;
      typedef typename mbd_types::contact_container            contact_container;
      typedef typename mbd_types::contact_type                 contact_type;

    protected:

      body_type         * m_A;          ///<
      body_type         * m_B;          ///<
      real_type           m_envelope;   ///<
      material_type     * m_material;   ///<
      contact_container * m_contacts;   ///<

    public:

      /**
       *
       */
      CollisionInfo(
          body_type * A
        , body_type * B
        , real_type const & envelope
        , material_type * material
        , contact_container * contacts        
        )
        : m_A(A)
        , m_B(B)
        , m_envelope(envelope)
        , m_material(material)
        , m_contacts( contacts )
      {}

    public:

      /**
       * Flip Bodies.
       * This method flip bodies A and B. This is usefull when
       * mirrowing an invokation of a collision handler.
       */
      void flip_bodies() 
      {
        body_type * tmp = m_A;
        m_A = m_B;
        m_B = tmp;
      }

      body_type         * get_body_A()   const { return m_A;        }
      body_type         * get_body_B()   const { return m_B;        }
      real_type const     get_envelope() const { return m_envelope; }
      material_type     * get_material() const { return m_material; }
      contact_container * get_contacts() const { return m_contacts; }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_COLLISION_INFO_H
#endif
