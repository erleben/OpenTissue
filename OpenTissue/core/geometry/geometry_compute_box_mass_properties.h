#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_BOX_MASS_PROPERTIES_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_BOX_MASS_PROPERTIES_H
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
  namespace geometry
  {

    /**
    * Compute Box Mass Properties
    *
    * Some density values in kg/m^3
    * Air     20 C, 1 atm, dry        1.21
    * Aluminum                      2700
    * Balsa wood                     120
    * Brick                         2000
    * Copper                        8900
    * Cork                           250
    * Diamond                       3300
    * Earth
    *   Average                     5500
    *   Core                        9500
    *   Crust                       2800
    * Glass                         2500
    * Gold                         19300
    * Helium (0 C, 1 atm)             0.178
    * Hydrogen (0C, 1 atm)            0.090
    * Ice                            917
    * Iron                          7900
    * Lead                         11300
    * Mercury                      13600
    * Nickel                        8800
    * Oil (olive)                    920
    * Oxygen (0 C, 1 atm)             1.43
    * Platinum                     21500
    * Silver                       10500
    * Styrofoam                      100
    * Tungsten                     19300
    * Uranium                      18700
    * Water
    *   20 C, 1 atm                 998
    *   20 C, 50 atm               1000
    *   seawater 20 C, 1 atm       1024
    *
    *
    * @param half_size          Half side extent of box
    * @param density         The mass density of the box.
    * @param mass            Upon return this argument holds the total mass of the box.
    * @param intertia        Upon return this argument holds the body frame inertia tensor.
    *                        Note this is a diagnoal 3x3 matrix, so only 3 values are needed.
    */
    template<typename real_type,typename vector3_type>
    void compute_box_mass_properties(
      vector3_type const & half_size,
      real_type const & density,
      real_type & mass,
      vector3_type & inertia
      )
    {
      real_type V = 8.*half_size(0)*half_size(1)*half_size(2);
      mass = density*V;
      inertia(0) = ((half_size(1)*half_size(1)+half_size(2)*half_size(2))*mass) / 3. ;
      inertia(1) = ((half_size(0)*half_size(0)+half_size(2)*half_size(2))*mass) / 3. ;
      inertia(2) = ((half_size(0)*half_size(0)+half_size(1)*half_size(1))*mass) / 3. ;
    }

  } //End of namespace geometry
} //End of namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_BOX_MASS_PROPERTIES_H
#endif
