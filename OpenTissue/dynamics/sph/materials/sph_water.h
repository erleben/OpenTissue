#ifndef OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_WATER_H
#define OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_WATER_H
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
    * SPH Water Material.
    */
    template< typename Types >
    class Water : public Material<Types>
    {
    public:
      typedef Material<Types>  base_type;
      typedef typename Types::real_type  real_type;

    public:
      /**
      * Default Constructor.
      */
      Water() : base_type()
      {
        base_type::m_name             = "Water";

        base_type::m_density          =    998.29;    // [kg/m^3]
        base_type::m_particle_mass    =      0.02;    // [kg]
        base_type::m_pressure         = 101325.0;     // [Pa = N/m^2 = kg/m.s^2]

        base_type::m_tension          =      0.0728;  // [N/m = kg/s^2]
        base_type::m_viscosity        =      3.5;     // [Pa.s = N.s/m^2 = kg/m.s]
        //      base_type::m_gas_stiffness    =      4.5;     // [J = N.m = kg.m^2/s^2]  // used for MCG03 symmetric pressure force
        base_type::m_gas_stiffness    =      3.0;     // [J = N.m = kg.m^2/s^2]  // used for DC96 symmetric pressure force

        // 0.01   ->   3.0
        // 0.001  -> 100.0

        base_type::m_timestep         =      0.01;    // [s]
        base_type::m_kernel_particles =     20.0;
        //      base_type::m_restitution      =      1.0;
        base_type::m_restitution      =      0.0;

        base_type::m_red              =      0.09;//0.0;
        base_type::m_green            =      0.31;//0.2;
        base_type::m_blue             =      0.98;//0.8;
      }

      /**
      * Deconstructor.
      */
      ~Water()
      {
      }

    private:


      /**
      * @return    the average distance between particles.
      */
      real_type dist() const
      {
        const real_type dV = base_type::m_volume/base_type::m_particles;
        //---- volume of sphere with radius r:  (4 pi *r^3) /3  =>  r =
        const real_type dist = std::pow(((3.*dV)/(4.0*math::detail::pi<real_type>())), 1./3.);
        return dist;
      }

      /**
      * @param pressure   holds the pressure in bar
      * @return           the number of molekyles in water volume.
      */
      real_type molekyles(const real_type& pressure)
      {
        //--- Assuming atmossphere pressure 1 bar, How
        //--- many molekyles would there be?
        const real_type P = base_type::Pa(pressure);
        const real_type molekyles = base_type::molekyles(P,base_type::m_volume,base_type::m_tempature);
        return molekyles;
      }

    }; // End class Water

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_MATERIALS_SPH_WATER_H
#endif
