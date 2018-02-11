//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2009 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_generate_PD.h>
#include <OpenTissue/utility/utility_timer.h>

//#define USE_ATLAS
#include <OpenTissue/core/math/big/big_svd.h>


int main( int argc, char **argv )
{
  typedef ublas::matrix<double>            matrix_type;
  typedef ublas::compressed_matrix<double> comp_matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef vector_type::size_type           size_type;

  size_type N = 10;  
  comp_matrix_type A;
  matrix_type U,V;
  vector_type s;
  
  OpenTissue::math::big::generate_PD(N,A);
  
  OpenTissue::utility::Timer<double> watch;
  watch.start();
  for(size_type iter=0;iter<1000;++iter)
  {
    OpenTissue::math::big::svd(A,U,s,V);
        
  }
  watch.stop();
  std::cout << "svd took " << watch() << " seconds" << std::endl;

  return 0;
}
