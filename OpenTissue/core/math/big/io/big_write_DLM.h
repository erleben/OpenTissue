#ifndef OPENTISSUE_CORE_MATH_BIG_IO_BIG_WRITE_DLM_H
#define OPENTISSUE_CORE_MATH_BIG_IO_BIG_WRITE_DLM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>

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
      * Save a matrix to a ASCII text file (Fortran format). See 
      * Matlab Function reference, dlmread, dlmwrite.
      *
      * @param A                 The matrix that should be written.
      * @param filename          The text filename, e.g., "A.dlm"
      * @return                  Whether the read operation succeed.
      */
      template<typename matrix_type>
      inline bool write_dlm_matrix(std::string const & filename, matrix_type &A)
      {
        using std::fabs;

        typedef typename matrix_type::value_type                   value_type;
        typedef          OpenTissue::math::ValueTraits<value_type> value_traits;

        static value_type const tiny = boost::numeric_cast<value_type>(1e-10);

        std::ofstream file;        
        file.open( filename.c_str() );
        if ( !file.is_open() || file.bad() )
        {
          throw std::logic_error("could not open the specified file");
          return false;
        }

        file << A.size1() << " " << A.size2() << " " << A.nnz() << std::endl;

        for(size_t i=0;i<A.size1();++i)
        {
          for(size_t j=0;j<A.size2();++j)
          {
            value_type value = A(i,j);
            if(value>value_traits::zero() || value<value_traits::zero())
              file << i << " " << j << " " << value << std::endl;
          }
        }

        file.flush();
        file.close();

        return true;
      }

      /**
      * Write a vector to a ASCII text file (Fortran format). Provides
      * a way to write a matlab space delimited matrix. See Matlab Function
      * reference, dlmread, dlmwrite.
      *
      * @param filename    The text filename, e.g., "x.dlm"
      * @param x           The vector that should be written.
      * @return            Whether the read operation succeed.
      */
      template<typename vector_type>
      inline bool write_dlm_vector(std::string const & filename, vector_type & x)
      {
        std::ofstream file;
        file.open( filename.c_str() );
        if ( !file.is_open() || file.bad() )
        {
          throw std::logic_error("could not open file?");
          return false;
        }
        file << x.size() << std::endl;
        for(size_t i = 0;i<x.size();++i)
        {
          file << i << " " << x(i) << std::endl;
        }
        file.flush();
        file.close();
        return true;
      }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_IO_BIG_WRITE_DLM_H
#endif
