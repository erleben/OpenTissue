#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>  

// 2007-10-28 kenny: io routines
#include <OpenTissue/core/math/big/io/big_matlab_write.h>
#include <OpenTissue/core/math/big/io/big_read_DLM.h>

// 2007-10-28 kenny: dummy stuff
#include <OpenTissue/core/math/big/big_identity_preconditioner.h>

// 2007-9-28 kenny: fast methods for working with ublas compressed matrices
#include <OpenTissue/core/math/big/big_prod.h>
#include <OpenTissue/core/math/big/big_prod_add.h>
#include <OpenTissue/core/math/big/big_prod_add_rhs.h>
#include <OpenTissue/core/math/big/big_prod_row.h>
#include <OpenTissue/core/math/big/big_prod_sub.h>
#include <OpenTissue/core/math/big/big_prod_sub_rhs.h>
#include <OpenTissue/core/math/big/big_prod_trans.h>
#include <OpenTissue/core/math/big/big_residual.h>

// 2007-10-12 kenny: Advanced tools for working with a Shur System
#include <OpenTissue/core/math/big/big_shur_system.h>

// 2007-9-28 kenny: Iterative method for solving systems of linear equations, A x = b
#include <OpenTissue/core/math/big/big_jacobi.h>
#include <OpenTissue/core/math/big/big_forward_gauss_seidel.h>
#include <OpenTissue/core/math/big/big_backward_gauss_seidel.h>
#include <OpenTissue/core/math/big/big_symmetric_gauss_seidel.h>
#include <OpenTissue/core/math/big/big_conjugate_gradient.h>
#include <OpenTissue/core/math/big/big_gmres.h>

// 2007-10-12 kenny: Iterative method for solving MCP problems
//#include <OpenTissue/core/math/big/big_lmcp_pgs.h>

// 2007-9-28 kenny: Direct methods for solving systems of linear equations, A x = b
#include <OpenTissue/core/math/big/big_svd.h>
#include <OpenTissue/core/math/big/big_lu.h>
#include <OpenTissue/core/math/big/big_cholesky.h>

// 2008-03-03 kenny: matrix generation and testing functionality
#include <OpenTissue/core/math/big/big_generate_random.h>
#include <OpenTissue/core/math/big/big_generate_PD.h>
#include <OpenTissue/core/math/big/big_generate_PSD.h>
#include <OpenTissue/core/math/big/big_gram_schmidt.h>
#include <OpenTissue/core/math/big/big_diag.h>
#include <OpenTissue/core/math/big/big_is_orthonormal.h>
#include <OpenTissue/core/math/big/big_is_symmetric.h>


// OPENTISSUE_CORE_MATH_BIG_BIG_H
#endif
