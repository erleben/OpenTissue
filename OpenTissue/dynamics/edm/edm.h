#ifndef OPENTISSUE_DYNAMICS_EDM_H
#define OPENTISSUE_DYNAMICS_EDM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

/**
 * Main EDM include file.
 *
 * EDM = Elastically Deformable Models (Terzopoulos/Kelager method).
 * 
 * This implementation is based upon
 * Paper: D. Terzopoulos, J. C. Platt, A. H. Barr, and K. Fleischer. "Elastically Deformable Models".
 *        Computer Graphics, Volume 21, Number 4, July 1987, pp. 205-214, 1987.
 *
 * The elastically deformable solids improvements are based upon
 * Paper: M. Kelager, A. Fleron, and K. Erleben. "Area and Volume Restoration in Elastically Deformable Solids".
 *        Electronic Letters on Compuer Vision and Image Analysis (ELCVIA), Vol. 5(3), pp. 32-43, 2005.
 */


/**
 * Recursively includes required headers.
 */
#include <OpenTissue/dynamics/edm/edm_force.h>
#include <OpenTissue/dynamics/edm/edm_model.h>
#include <OpenTissue/dynamics/edm/edm_object.h>
#include <OpenTissue/dynamics/edm/edm_solid.h>
#include <OpenTissue/dynamics/edm/edm_surface.h>
#include <OpenTissue/dynamics/edm/edm_system.h>
#include <OpenTissue/dynamics/edm/edm_types.h>

#include <OpenTissue/dynamics/edm/forces/edm_forces.h>
#include <OpenTissue/dynamics/edm/models/edm_models.h>


// OPENTISSUE_DYNAMICS_EDM_H
#endif
