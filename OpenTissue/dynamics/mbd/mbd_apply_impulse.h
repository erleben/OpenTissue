#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_APPLY_IMPULSE_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_APPLY_IMPULSE_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Apply Impulse to body_type.
    *
    * @param body    The body where the impulse should be applied to.
    * @param r       The arm from the center of mass of the body to the point of contact, where the impulse is applied.
    * @param J       The impulse that should be applied.
    */
    template<typename body_type>
      void apply_impulse(body_type * body, typename body_type::vector3_type const & r, typename body_type::vector3_type const & J)
    {
      typedef typename body_type::real_type       real_type;
      typedef typename body_type::vector3_type    vector3_type;
      typedef typename body_type::matrix3x3_type  matrix3x3_type;

      vector3_type v,w,dw;
      real_type inv_m = body->get_inverse_mass();
      body->get_velocity(v);
      v += J*inv_m;
      body->set_velocity(v);

      dw = cross(r , J);
      body->get_spin(w);
      matrix3x3_type invI;
      body->get_inverse_inertia_wcs(invI);
      w += invI*dw;
      body->set_spin(w);
    }

  } //End of namespace mbd
} //End of namespace OpenTissue
#endif // OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_APPLY_IMPULSE_H
