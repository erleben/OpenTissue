#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_IDEAL_GAS_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_IDEAL_GAS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * Implements the Ideal Gas.
    * Govering equation is:
    *
    *    P V = n R T
    *
    * Some numbers:
    *
    *   Molare volume of water: V_m  = 0.000055 m^3 / mol
    *   Molar mass of water:    m_m  =  0.018 Kg/mol
    *
    * Some units:
    *
    *   Newton:  N = Kg m/s^2             (Unit of force mass-acceleration)
    *   Joule:   J = Kg m^2 / s^2 = N m   (The energy equivalent to dragging a one kilogram mass a distance of one meter).
    *   Pascal : Pa = N/m^2
    */
    template<typename real_type>
    class IdealGas
    {
    public:

      /**
      * Compute mol.
      *
      * Solves
      *
      *         P V
      *   n = -------
      *         R T
      *
      * @param P   Pressure     (N/m^2)
      * @param V   Volume       (m^3)
      * @param T   Temparature  (K)
      *
      * @return    The number of mol.
      */
      real_type mol(real_type P,real_type V,real_type T) const
      {
        assert(V>0);
        assert(T>0);
        real_type R = static_cast<real_type>(8.314510);   ///< Gas constant 8.314510  (J /mol K)
        return (P*V) / (R*T);
      }

      /**
      * Compute mol.
      *
      * @param V    Volume (m^3)
      * @param Vm   Molar volume (m^3/mol)
      *
      * @return    The number of mol.
      */
      real_type mol(real_type V,real_type molar_volume) const
      {
        assert(V>0 && molar_volume>0);
        return V/molar_volume;
      }

      /**
      * Compute Molekyles
      *
      * @param P   Pressure     (N/m^2)
      * @param V   Volume       (m^3)
      * @param T   Temparature  (K)
      *
      * @return    The number of molekyles.
      */
      real_type molekyles(real_type P,real_type V,real_type T) const
      {
        assert(V>0);
        assert(T>0);
        real_type Na = static_cast<real_type>(6.0221367e23);  ///< Avogadros number 6.0221367 10^23 1/mol, number of molecyles per mol.
        return mol(P,V,T)*Na;
      }

      /**
      * Compute Mass
      *
      * @param mol          The number of mol.
      * @param molar_mass   The molar mass of the gas (Kg/mol).
      *
      * @return   The total mass (Kg)
      */
      real_type mass(real_type mol, real_type molar_mass) const
      {
        assert(mol>0 && molar_mass>0);
        return mol*molar_mass;
      }

      /**
      * Temparature unit-conversion.
      *
      * @param K  temparture in Kelvin
      *
      * @return   temparature in Celsius
      */
      real_type celsius(real_type K) const
      {
        return K - static_cast<real_type>(273.15);
      }

      /**
      * Temparature unit-conversion.
      *
      * @param C  temparture in Celsius
      *
      * @return   temparature in Kelvin
      */
      real_type kelvin(real_type C) const
      {
        return C + static_cast<real_type>(273.15);
      }

      /**
      * Unit conversion Bar to Pascal
      */
      real_type bar (real_type Pa) const
      {
        return Pa/static_cast<real_type>(1e5);
      }

      /**
      * Unit conversion Bar to Pascal
      */
      real_type Pa (real_type bar) const
      {
        return bar*static_cast<real_type>(1e5);
      }

      /**
      * Unit conversion Pa to kPa
      */
      real_type kPa(real_type Pa) const
      {
        return Pa/static_cast<real_type>(1e3);
      }

      /**
      * Compute molar mass.
      *
      * @param rho   Density (Kg/m^3)
      * @param V     Volume  (m^3)
      * @param mol   mol
      *
      * return       The molar mass (Kg/mol)
      */
      real_type molar_mass(real_type rho, real_type V, real_type mol) const
      {
        assert(V>0 && rho>0 && mol>0);
        return (rho*V)/mol;
      }

    };

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_IDEAL_GAS_H
#endif
