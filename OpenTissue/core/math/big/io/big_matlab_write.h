#ifndef OPENTISSUE_CORE_MATH_BIG_IO_BIG_MATLAB_WRITE_H
#define OPENTISSUE_CORE_MATH_BIG_IO_BIG_MATLAB_WRITE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <iosfwd>
#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {
      /**
      * Vector Stream Output Operator.
      *
      *  @param os      The stream to output the vector to.
      *  @param v       A vector expression.
      *
      * @return        The stream that the vector was written to.
      */
      template<class E, class T, class VE>
      inline std::basic_ostream<E, T> & operator << (
        std::basic_ostream<E, T> &os
        , ublas::vector_expression<VE> const & v) 
      {
        typedef typename VE::size_type size_type;
        size_type size = v ().size ();
        std::basic_ostringstream<E, T, std::allocator<E> > s;
        s.flags (os.flags ());
        s.imbue (os.getloc ());
        s.precision (os.precision ());
        s << '[';
        if (size > 0)
          s << v () (0);
        for (size_type i = 1; i < size; ++ i)
          s << ',' << v () (i);
        s << ']';
        return os << s.str ().c_str ();
      }

      /**
      * Compressed Matrix Stream Output Operator.
      *
      * @param os   The output stream the matrix should be output to.
      * @param A    Compressed Matrix to output
      *
      * @return        The stream that the matrix was written to.
      */
      template<class T, class F>
      inline std::ostream & operator << (
        std::ostream & os
        , ublas::compressed_matrix<T, F> const & A
        )
      {
        //--- get dimensions
        unsigned int m = static_cast<unsigned int>(A.size1());
        unsigned int n = static_cast<unsigned int>(A.size2());
        unsigned int nzeros = 0;
        unsigned int idx = 0;
        nzeros = 0;
        for(unsigned int i=0;i<A.size1();++i)
          for(unsigned int j=0;j<A.size2();++j)
            if(A(i,j))
              ++nzeros;

        //--- allocate space...
        ublas::vector<unsigned int> idx1(nzeros);
        ublas::vector<unsigned int> idx2(nzeros);
        ublas::vector<T> value(nzeros);

        for(unsigned int i=0;i<A.size1();++i)
          for(unsigned int j=0;j<A.size2();++j)
            if(A(i,j))
            {
              idx1(idx) = i + 1;
              idx2(idx) = j + 1;
              value(idx) = A(i,j);
              ++idx;
            }
            //--- create matlab output
            os << "sparse(" << idx1 << "," << idx2 << "," << value << "," << m << "," << n << ")";
            return os;         
      };

      /**
      * General Matrix Stream Output Operator.
      *
      * @param os      The stream that the matrix should be output to.
      * @param m       The matrix that should be outputted.
      *
      * @return        The stream that the matrix was output to.
      */
      template<class E, class T, class ME>
      inline std::basic_ostream<E, T> & operator << (
        std::basic_ostream<E, T> & os
        , ublas::matrix_expression<ME> const & m
        ) 
      {
        typedef typename ME::size_type size_type;
        size_type size1 = m ().size1 ();
        size_type size2 = m ().size2 ();
        std::basic_ostringstream<E, T, std::allocator<E> > s;
        s.flags (os.flags ());
        s.imbue (os.getloc ());
        s.precision (os.precision ());
        s << '[';
        if (size1 > 0) {
          s << '[' ;
          if (size2 > 0)
            s << m () (0, 0);
          for (size_type j = 1; j < size2; ++ j)
            s << ',' << m () (0, j);
          s << ']';
        }
        for (size_type i = 1; i < size1; ++ i) {
          s << ";[" ;
          if (size2 > 0)
            s << m () (i, 0);
          for (size_type j = 1; j < size2; ++ j)
            s << ',' << m () (i, j);
          s << ']';
        }
        s << ']';
        return os << s.str ().c_str ();
      }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_IO_BIG_MATLAB_WRITE_H
#endif



