#ifndef BENCHMARK_H
#define BENCHMARK_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include "generate_problem.h"

#include <OpenTissue/utility/utility_timer.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/io/big_matlab_write.h>
#include <OpenTissue/core/math/optimization/optimization_bfgs.h>

#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>

typedef double real_type;
typedef ublas::compressed_matrix<real_type> matrix_type;
typedef ublas::vector<real_type>            vector_type;
typedef vector_type::size_type              size_type;
typedef ublas::identity_matrix<real_type>   identity_matrix_type;
typedef ublas::vector<size_t>               idx_vector_type;

class F
{
public:
  matrix_type const & m_A;
  vector_type const & m_b;

  F(matrix_type const & A, vector_type const & b)
    : m_A(A)
    , m_b(b)
  {}

  real_type operator()( vector_type const & x ) const
  {
    return ublas::inner_prod(x, ublas::prod(m_A,x)) - inner_prod(m_b, x);
  }
};

class nabla_F
{
public:
  matrix_type const & m_A;
  vector_type const & m_b;

  nabla_F(matrix_type const & A, vector_type const & b)
    : m_A(A)
    , m_b(b)
  {}

  vector_type operator()( vector_type const & x ) const
  {
    return  vector_type( 2*ublas::prod(m_A,x) - m_b );
  }


  void operator()( vector_type const & x, vector_type & g) const
  {
    ublas::noalias(g) = ( 2*ublas::prod(m_A,x) - m_b );
  }

};

/**
* Benchmark Utility Function.
*
* @param N            The problem size.
* @param T            Number of test cases that should be run (i.e. problems to solve).
*/
inline void benchmark( 
                        std::string const & prefix
                      , std::string const & matlab_filename
                      , std::string const & latex_filename
                      , size_t const & N
                      , size_t const & T
                      )
{
  using namespace OpenTissue::math::big;
  using std::min;
  using std::max;

  std::cout << "--- started benchmark --- " << prefix << std::endl;

  std::ofstream latex_file( latex_filename.c_str(), std::ios::out | std::ios::app);
  if (!latex_file)
  {
    std::cerr << "benchmark: Error unable to create file: "<< latex_filename << std::endl;
    return;
  }
  latex_file << "%% ------ " << prefix << " -------------" << std::endl;


  std::ofstream matlab_file( matlab_filename.c_str(), std::ios::out);
  if (!matlab_file) // TODO: henrikd - gcc says "always true"
  {
    std::cerr << "benchmark: Error unable to create file: "<< matlab_filename << std::endl;
    return;
  }
  matlab_file << "close all;" << std::endl;
  matlab_file << "clear all;" << std::endl;

  matrix_type stats;
  stats.resize(T,2,false);

  idx_vector_type cnt_status;
  cnt_status.resize(7,false);
  cnt_status.clear();

  double min_time = 10000.0;
  double max_time = 0.0;
  double avg_time = 0.0;

  size_t min_iterations = 10000;
  size_t max_iterations = 0;
  double avg_iterations = 0.0;
  size_t cnt_absolute_converged = 0;

  matlab_file << "filename1 = '" << prefix << "_convergence';" << std::endl;
  matlab_file << "figure(1);" << std::endl;
  matlab_file << "clf" << std::endl;
  matlab_file << "set(gca,'fontsize',18);" << std::endl;
  matlab_file << "hold on;" << std::endl;

  for(size_t tst=0;tst<T;++tst)
  {
    matrix_type A;
    vector_type x,b,l,u, profile;

    generate_problem(N, A, b, l, u ); 

    F f(A,b);
    nabla_F nabla_f(A,b);

    size_type max_iteration_limit  = 1000;
    real_type absolute_tolerance   = boost::numeric_cast<real_type>(1e-6);
    //real_type relative_tolerance   = boost::numeric_cast<real_type>(0.000000001);
    //real_type stagnation_tolerance = boost::numeric_cast<real_type>(0.000000001);
    real_type relative_tolerance   = boost::numeric_cast<real_type>(0.0);
    real_type stagnation_tolerance = boost::numeric_cast<real_type>(0.0);
    size_t status                  = 0;
    size_type iteration            = 0;
    real_type accuracy             = boost::numeric_cast<real_type>(0.0);

    real_type alpha                = boost::numeric_cast<real_type>(0.0001);
    real_type beta                 = boost::numeric_cast<real_type>(0.5);

    x.resize(N, false);
    x.clear();
    matrix_type H;
    H.resize(N,N,false);

    identity_matrix_type I(N,N);

    // use H = I/4, and x = 0
    // use H = I, and x = 0
    // use H = 4*I, and x = 0
    // use H = I/4, and x = random
    // use H = I, and x = random
    // use H = 4*I, and x = random
    // use H = random PD, and x = 0
    // use H = random PD, and x = random
    // H = g g^T, x = 0
    // H = g g^T, x = random
    // H = exact Hessian, x = solution!
    H = I;
    x.clear();

    OpenTissue::utility::Timer<double> duration;

    duration.start();

    OpenTissue::math::optimization::bfgs(
      f
      , nabla_f
      , H
      , x 
      , max_iteration_limit
      , absolute_tolerance
      , relative_tolerance
      , stagnation_tolerance
      , status
      , iteration
      , accuracy
      , alpha
      , beta
      , &profile
      );

    duration.stop();

    cnt_status(status) =  cnt_status(status) + 1;
    stats(tst, 0) = iteration;
    stats(tst, 1) = duration();

    if(status == OpenTissue::math::optimization::ABSOLUTE_CONVERGENCE)
    {
      ++cnt_absolute_converged;

      min_time = min( duration(), min_time );
      max_time = max( duration(), max_time );
      avg_time += duration();

      min_iterations = min( iteration, min_iterations );
      max_iterations = max( iteration, max_iterations );
      avg_iterations += iteration;
    }
    if(iteration>0)
    {
      matlab_file << "P" << tst << " = " << profile << ";" << std::endl;
      size_t choice = (tst%6);
      std::string color = "";
      switch(choice)
      {
      case 0: color = ",'b'"; break;
      case 1: color = ",'g'"; break;
      case 2: color = ",'r'"; break;
      case 3: color = ",'c'"; break;
      case 4: color = ",'m'"; break;
      case 5: color = ",'y'"; break;
      };
      matlab_file << "plot(P"<<tst<<"(1:"<< iteration <<")"<< color <<");" << std::endl;
    }
  }
  matlab_file << "axis tight;" << std::endl;
  matlab_file << "xlabel('Iterations','fontsize',18);" << std::endl;
  matlab_file << "ylabel('\\theta','fontsize',18);" << std::endl;
  matlab_file << "hold off;" << std::endl;
  matlab_file << "print('-f1','-depsc2', filename1);" << std::endl;
  matlab_file << "print('-f1','-dpng', filename1);" << std::endl;


  if(cnt_absolute_converged>0)
  {
    avg_time      /= cnt_absolute_converged;
    avg_iterations /= cnt_absolute_converged;

    latex_file << "Size" << " & "  << "Min (secs)"  << " & " << "Avg (secs)"  << " & " <<  "Max (secs)" << "\\\\" << std::endl;
    latex_file << N << " & "  << min_time       << " & " << avg_time       << " & " <<  max_time << "\\\\" << std::endl;
    latex_file << "Size" << " & "  << "Min (\\#)"  << " & " << "Avg (\\#)"  << " & " <<  "Max (\\#)" << "\\\\" << std::endl;
    latex_file << N << " & " << min_iterations << " & " << avg_iterations << " & " <<  max_iterations << "\\\\" << std::endl;

    std::cout << "time (min,avg,max) : " << min_time       << " : " << avg_time       << " : " <<  max_time << std::endl;
    std::cout << "iter (min,avg,max) : " << min_iterations << " : " << avg_iterations << " : " <<  max_iterations << std::endl;
    std::cout << std::endl;
  }

  for(size_t i=0;i<cnt_status.size();++i)
    std::cout << cnt_status(i) << "\t:\t"<< OpenTissue::math::optimization::get_error_message( i ) << std::endl;
  std::cout << std::endl;
  latex_file << N << " & ";
  for(size_t i=0;i<cnt_status.size();++i)
    latex_file << cnt_status(i) <<  " & " ;
  latex_file << std::endl;

  matlab_file << "S = " << stats << ";" << std::endl;
  matlab_file << "S1 = full( S(:, 1) );" << std::endl;
  matlab_file << "S2 = full( S(:, 2) );" << std::endl;

  matlab_file << "s1_txt = 'Iterations';" << std::endl;
  matlab_file << "s2_txt = 'Duration (seconds)';" << std::endl;

  matlab_file << "filename2 = '" << prefix << "iterations';" << std::endl;
  matlab_file << "filename3 = '" << prefix << "duration';" << std::endl;

  matlab_file << "figure(2);" << std::endl;
  matlab_file << "clf" << std::endl;
  matlab_file << "set(gca,'fontsize',18);" << std::endl;
  matlab_file << "hold on;" << std::endl;
  matlab_file << "hist(S1);" << std::endl;
  matlab_file << "xlabel(s1_txt,'fontsize',18);" << std::endl;
  matlab_file << "ylabel('Count','fontsize',18);" << std::endl;
  matlab_file << "hold off;" << std::endl;
  matlab_file << "print('-f2','-depsc2', filename2);" << std::endl;
  matlab_file << "print('-f2','-dpng', filename2);" << std::endl;

  matlab_file << "figure(3);" << std::endl;
  matlab_file << "clf" << std::endl;
  matlab_file << "set(gca,'fontsize',18);" << std::endl;
  matlab_file << "hold on;" << std::endl;
  matlab_file << "hist(S2);" << std::endl;
  matlab_file << "xlabel(s2_txt,'fontsize',18);" << std::endl;
  matlab_file << "ylabel('Count','fontsize',18);" << std::endl;
  matlab_file << "hold off;" << std::endl;
  matlab_file << "print('-f3','-depsc2', filename3);" << std::endl;
  matlab_file << "print('-f3','-dpng', filename3);" << std::endl;

  matlab_file.flush();
  matlab_file.close();

  latex_file.flush();
  latex_file.close();
  std::cout << "done writting m file.." << std::endl;
}

// BENCHMARK_H
#endif
