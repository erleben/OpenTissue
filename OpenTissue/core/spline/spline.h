#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/spline/spline_math_types.h>
#include <OpenTissue/core/spline/spline_nubs.h>
#include <OpenTissue/core/spline/spline_initialize_m_table.h>
#include <OpenTissue/core/spline/spline_compute_knot_span.h>
#include <OpenTissue/core/spline/spline_compute_basis.h>
#include <OpenTissue/core/spline/spline_compute_nonzero_basis.h>

#include <OpenTissue/core/spline/spline_solve_tridiagonal.h>
#include <OpenTissue/core/spline/spline_compute_chord_length_knot_vector.h>
#include <OpenTissue/core/spline/spline_compute_basis_derivatives.h>
#include <OpenTissue/core/spline/spline_compute_spline_derivatives.h>
#include <OpenTissue/core/spline/spline_compute_point_on_spline.h>
#include <OpenTissue/core/spline/spline_insert_knot.h>

#include <OpenTissue/core/spline/spline_make_periodic.h>
#include <OpenTissue/core/spline/spline_make_cubic_interpolation.h>

//OPENTISSUE_CORE_SPLINE_SPLINE_H
#endif
