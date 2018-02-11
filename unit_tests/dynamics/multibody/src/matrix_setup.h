#ifndef MATRIX_SETUP_H
#define MATRIX_SETUP_H
//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp>

#include <ctime>   // for std::time()
#include <cstdlib> // for std::srand() and std::rand(), defaults to this if no boost!

template<typename size_type, typename matrix_type>
void matrix_setup(size_type bodies, size_type contacts, matrix_type & W, matrix_type & J)
{
  typedef matrix_type::value_type  real_type; 

  std::srand(static_cast<unsigned int>(std::time(0)));

  // Allocate matrices and fill out the non-zero patterns of J and W
  size_type m = 3*contacts;
  size_type n = 6*bodies;

  J.resize(m,n,false);
  W.resize(n,n,false);

  // W is a 3x3 blocked diagonal matrix
  for(size_type i=0;i<bodies;++i)
  {
    size_type j = i*6;

    W(j,j)     = 1.0;
    W(j+1,j+1) = 1.0;
    W(j+2,j+2) = 1.0;
    W(j+3,j+3) = 1.0;    W(j+3,j+4) = 0.5;    W(j+3,j+5) = 0.5;
    W(j+4,j+3) = 0.5;    W(j+4,j+4) = 1.0;    W(j+4,j+5) = 0.5;
    W(j+5,j+3) = 0.5;    W(j+5,j+4) = 0.5;    W(j+5,j+5) = 1.0;
  }

  // J is a 3x3 blocked matrix too with the property that every row has exactly 12 non-zero entries!
  for(size_type i=0;i<contacts;++i)
  {
    size_type j = i*3;

    double rnd = rand()/(1.0*RAND_MAX);
    size_type tmp = boost::numeric_cast<size_type>( bodies*rnd );

    size_type b1 = tmp%bodies;
    size_type b2 = (tmp+1)%bodies;

    b1 *= 6;
    b2 *= 6;

    J(j  ,b1) = 1.0;    J(j  ,b1+1) = 1.0;  J(j  ,b1+2) = 1.0; J(j  ,b1+3) = 1.0; J(j  ,b1+4) = 1.0; J(j  ,b1+5) = 1.0;
    J(j+1,b1) = 1.0;    J(j+1,b1+1) = 1.0;  J(j+1,b1+2) = 1.0; J(j+1,b1+3) = 1.0; J(j+1,b1+4) = 1.0; J(j+1,b1+5) = 1.0;
    J(j+2,b1) = 1.0;    J(j+2,b1+1) = 1.0;  J(j+2,b1+2) = 1.0; J(j+2,b1+3) = 1.0; J(j+2,b1+4) = 1.0; J(j+2,b1+5) = 1.0;

    J(j  ,b2) = 1.0;    J(j  ,b2+1) = 1.0;  J(j  ,b2+2) = 1.0; J(j  ,b2+3) = 1.0; J(j  ,b2+4) = 1.0; J(j  ,b2+5) = 1.0;
    J(j+1,b2) = 1.0;    J(j+1,b2+1) = 1.0;  J(j+1,b2+2) = 1.0; J(j+1,b2+3) = 1.0; J(j+1,b2+4) = 1.0; J(j+1,b2+5) = 1.0;
    J(j+2,b2) = 1.0;    J(j+2,b2+1) = 1.0;  J(j+2,b2+2) = 1.0; J(j+2,b2+3) = 1.0; J(j+2,b2+4) = 1.0; J(j+2,b2+5) = 1.0;
  }
}

// MATRIX_SETUP_H
#endif
