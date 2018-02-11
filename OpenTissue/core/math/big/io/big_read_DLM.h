#ifndef OPENTISSUE_CORE_MATH_BIG_IO_READ_DLM_H
#define OPENTISSUE_CORE_MATH_BIG_IO_READ_DLM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp> // Needed for boost::numeric_cast

#include <fstream>
#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {
      /**
      * Load a matrix from a ASCII text file (Fortran format)
      * C++ Interface: ReadDLMmatrix
      *
      * Description:  Provides a way to read a matlab space delimited matrix
      *
      *
      * See Matlab Function reference, dlmread, dlmwrite.
      *
      *
      * Author: Ricardo Ortiz <rortizro@math.uiowa.edu>, (C) 2006
      *
      * Copyright: See COPYING file that comes with this distribution
      *
      * @param m The matrix(vector) with compatible interface to read in.
      * @param filename The text filename, e.g., "A.dlm"
      * @return Whether the read operation succeed.
      */
      template<typename matrix_type>
      inline bool read_dlm_matrix(std::string const & filename, matrix_type &A)
      {
        using std::fabs;

        typedef typename matrix_type::value_type  real_type;
        typedef typename matrix_type::size_type   size_type;
        std::ifstream ifs;        
        ifs.open( filename.c_str() );
        if ( !ifs.is_open() || ifs.bad() )
        {
          throw std::logic_error("could not open the specified file");
          return false;
        }
        size_type m,n,nnz;
        ifs >> m; ifs >> n; ifs >> nnz;       
        matrix_type M(m,n,nnz); // 2007-05-29 kenny: why not use A directly?
        static real_type const tiny = boost::numeric_cast<real_type>(1e-10);
        size_type i, j;
        while ( ifs.good() )
        {
          ifs >> i;
          ifs >> j;
          real_type s;
          ifs >> s;
          if( fabs(s) > tiny) 
            M(i,j) = s;
        }
        A = M;
        return true;
      }

      /**
      * Load a vector from a ASCII text file (Fortran format)
      * C++ Interface: ReadDLMmatrix
      *
      * Description:  Provides a way to read a matlab space delimited matrix
      *
      * See Matlab Function reference, dlmread, dlmwrite.
      *
      * Author: Ricardo Ortiz <rortizro@math.uiowa.edu>, (C) 2006
      * Copyright: See COPYING file that comes with this distribution
      *
      * @param m The vector with compatible interface to read in.
      * @param filename The text filename, e.g., "A.dlm"
      * @return Whether the read operation succeed.
      */
      template<typename vector_type>
      inline bool read_dlm_vector(std::string const & filename, vector_type & x)
      {
        typedef typename vector_type::size_type  size_type;

        std::ifstream ifs;
        
        ifs.open( filename.c_str() );

        if ( !ifs.is_open() || ifs.bad() )
        {
          throw std::logic_error("could not open file?");
          return false;
        }

        size_type i, size;

        ifs >> size;
        x.resize(size);

        while ( ifs.good() )
        {
          ifs >> i;
          ifs >> x(i);
        }
        return true;
      }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_IO_READ_DLM_H
#endif
