#ifndef OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_PARTICLE_H
#define OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_PARTICLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace meshless_deformation
  {
    namespace detail
    {

      template<typename math_types>
      class Particle
      {
      public:

        typedef typename math_types::value_traits     value_traits;
        typedef typename math_types::real_type        real_type;
        typedef typename math_types::vector3_type     vector3_type;
        typedef typename math_types::matrix3x3_type   matrix3x3_type;

      private:

        vector3_type * m_x;      ///< Current position.

      public: //should be protected?

        vector3_type m_x0;       ///< Original position.
        vector3_type m_v;       ///< Current velocity.

        vector3_type m_f_ext;  ///< External forces.typename
        vector3_type m_f_goal; ///< Goal forces (accumulator of goal positions for all clusters).
        real_type    m_mass;   ///< Mass of particle.
        bool         m_fixed;  ///< Boolean flag indicating if particle is fixed.

      public: //should be protected?

        //--- Shared by all clusters (if not clusters could be computed in parallel!)

        vector3_type m_g;      ///< Goal position (temporary placeholder for goal position in a single cluster).
        vector3_type m_q[3];   ///< relative location (wrt center of mass) of orignal positions.
        ///< q[0] is linear shear and stretch mode: [q_x, q_y, q_z]
        ///< q[1] is bend mode: [q_x^2, q_y^2, q_z^2]
        ///< q[2] is twist mode: [q_x q_y , q_y q_z, q_z q_x]
        vector3_type m_p;      ///< relative location (wrt center of mass) of orignal positions.

      public:

        Particle()
          : m_x(0)
          , m_v( value_traits::zero(),value_traits::zero(),value_traits::zero() )
          , m_mass( value_traits::one() )
          , m_fixed(false)
        {}

      public:

        bool & fixed() { return m_fixed; }

        real_type & mass() { return m_mass; }

        vector3_type & f_ext() { return m_f_ext; }

        void bind(vector3_type & x)    {      m_x = &x;  m_x0 = x;    }

        vector3_type & x(){  return *m_x; }

        vector3_type & v(){  return m_v; }

      };

    } // namespace detail
  } // namespace meshless_deformation
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_PARTICLE_H
#endif
