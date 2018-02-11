#ifndef GENERATE_PROBLEM_H
#define GENERATE_PROBLEM_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_generate_PD.h>
#include <OpenTissue/core/math/big/big_generate_PSD.h>
#include <OpenTissue/core/math/big/big_generate_random.h>

inline void generate_problem(
                             size_t const & n
                             , ublas::compressed_matrix<double> & A
                             , ublas::vector<double> & b
                             , ublas::vector<double> & l
                             , ublas::vector<double> & u
                             )
{
  A.resize(n,n,false);
  b.resize(n,false);
  l.resize(n,false);
  u.resize(n,false);
  OpenTissue::math::big::generate_random( n, b);
  OpenTissue::math::big::generate_random( n, u);
  l.assign( -u );
  OpenTissue::math::big::fast_generate_PD(n,A);
}

// GENERATE_PROBLEM_H
#endif
