#ifndef OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_STEAM_H
#define OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_STEAM_H
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
    * SPH Steam Material.
    */
    template< typename Types >
    class Steam : public Material<Types>
    {
    public:
      typedef Material<Types>  base_type;
      typedef typename Types::real_type  real_type;

    public:
      /**
      * Default Constructor.
      */
      Steam() : base_type()
      {
        base_type::m_name             = "Steam";

        base_type::m_density          =      0.59;    // [kg/m^3]
        base_type::m_particle_mass    =      0.00005; // [kg]
        base_type::m_pressure         = 101325.0;     // [Pa = N/m^2 = kg/m.s^2]

        base_type::m_buoyancy         =      5.0;     // [??]
        base_type::m_viscosity        =      0.01;    // [Pa.s = N.s/m^2 = kg/m.s]
        //      base_type::m_gas_stiffness    =      8.0;     // [J = N.m = kg.m^2/s^2]
        base_type::m_gas_stiffness    =      4.0;     // [J = N.m = kg.m^2/s^2]

        base_type::m_timestep         =      0.01;    // [s]
        base_type::m_kernel_particles =     12.0;
        base_type::m_restitution      =      0.0;

        base_type::m_red              =      0.75;
        base_type::m_green            =      0.75;
        base_type::m_blue             =      0.75;
      }

      /**
      * Deconstructor.
      */
      ~Steam()
      {
      }

    }; // End class Steam

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_STEAM_H
#endif
