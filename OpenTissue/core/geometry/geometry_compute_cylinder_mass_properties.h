#ifndef OPENTISSUE_DYNAMICS_MBD_PRIMITIVES_H
#define OPENTISSUE_DYNAMICS_MBD_PRIMITIVES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Compute Cylinder Mass Properties.
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
    * @param radius
    * @param half_height
    * @param density         The mass density of the cylinder.
    * @param mass            Upon return this argument holds the total mass of the cylinder.
    * @param intertia        Upon return this argument holds the body frame inertia tensor.
    *                        Note this is a diagnoal 3x3 matrix, so only 3 values are needed.
    */
    void compute_cylinder_mass_properties(
      real_type const & radius,
      real_type const & half_height,
      real_type const & density,
      real_type & mass,
      vector3_type & inertia
      )
    {
      real_type V = 2*math::detail::pi<real_type>()*radius*radius*half_height;
      mass = density*V;
      inertia(0) = inertia(2) = (radius*radius*.25 + half_height*half_height*.75)*mass;
      inertia(1) = .5*mass*radius*radius ;
    }

  } //End of namespace geometry
} //End of namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_PRIMITIVES_H
#endif
