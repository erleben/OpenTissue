#ifndef OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_MUCUS_H
#define OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_MUCUS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_material.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * SPH Mucus Material.
    *
    */
    template< typename Types >
    class Mucus : public Material<Types>
    {
    public:
      typedef Material<Types>  base_type;
      typedef typename Types::real_type  real_type;

    public:
      /**
      * Default Constructor.
      */
      Mucus() : base_type()
      {
        base_type::m_name             = "Mucus";

        base_type::m_density          = 1000.0;
        base_type::m_pressure         =    1.0;
        base_type::m_particle_mass    =    0.04;

        base_type::m_tension          =    6.0;
        base_type::m_viscosity        =   36.0;
        //      base_typem_gas_stiffness    =    7.0;
        base_type::m_gas_stiffness    =    5.0;

        base_type::m_timestep         =    0.01;
        base_type::m_kernel_particles =   40.0;
        base_type::m_restitution      =    0.5;

        base_type::m_red              =    0.06;
        base_type::m_green            =    0.8;
        base_type::m_blue             =    0.4;
      }

    }; // End class Mucus

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_MUCUS_H
#endif
