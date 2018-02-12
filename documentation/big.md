# Why Use the BIG Library?

The BIG library is a matrix library for large scale matrices (i.e. big matrices). BIG contains a lot of matrix equation solvers, LU,
SVD, conjugate gradient, Gauss-Seidel, Jacobi, gmres, Cholesky factorization. BIG also contains a lot of hand-written specialized prod-functions. These exploit the structure of ublas::compressed_matrix to achieve higher performance. IO routines support both Matlab output and DLM input. Needless to say every part of OpenTissue that used boost_matrix_solvers (the former large-scale matrix library used in OpenTissue) have been re factored to use the new BIG library. Advantages of BIG over its predecessor boost_matrix_solvers:

* BIG got extensive unit-testing
* BIG has been written to support free template functions as well as functors
* BIG contains a lot more functionality than its predecessor

# Purpose of the BIG Library

In OpenTissue we make the distinction between small-sized fix-size matrices and vectors and large-scale matrices and vectors.

By default OpenTissue uses the Boost uBLAS matrix vector library for dealing with large-scale matrices and vectors. However, Boost uBLAS does not always provide a high-level interface or easy to use interface allowing one to merely invoke for instance a conjugate gradient solver or some other matrix solver for that matter.

In some cases one need high-performance implementations of certain algorithms such as the LU and SVD routines in Lapack. In these cases one would use the so called boost bindings together with boost uBLAS in order to call routines deep inside other third-pary libraries (such as ATLAS) without even noticing it from ones C++ code.

The BIG library was written to provide the glue that binds everything together in a seamlessly transparent way. The BIG library was written to provide end-users with a variarity of sparse-matrix solvers and specialized routines for row-format compressed matrices.

# Overview of BIG Library

The BIG library contains quite many header files and is constantly expanding. In most cases the naming-convention of the header files in the BIG library should make it clear what the content of the header file is all about. We refer the reader to the source documentation (ie. doxygen and inline comments) for more information about the specifics. In the following we will merely give a brief overview of some of the most fundamental parts of the BIG library.
* A BIG include all header. This header file OpenTissue/math/big/big.h includes nearly all other files in the BIG library. It is intended as a convenience header file to be used by end-users that do not want to know any details about the header files that makes up the complete BIG library.
* A BIG types include header. This header file, OpenTissue/math/big/big_types.h, takes care of including appropriate header files from Boost uBLAS. It makes sure that one has access to often used types. In most cases this includes

      ublas::matrix<...>
      ublas::compresed_matrix<...>
      ublas::vector<...>

  The header file also defines the convenience name space ublas (used in the code above) for ease of use. Besides some basic IO support for streamed matlab output is set up and the Boost Bindings are also included.
* A lot of ublas::compressed_matrix routines have been written. In most cases end users could use the ublas::axpy() method instead. In most cases the only difference the specialized routines and the ublas::axpy()-method is that axpy uses expression templates. The specialized routines work directly on the raw data of the arguments passed to them.

* As of this writing the set of specialized routines can be find in the following header files.
  - OpenTissue/math/big/big_prod_trans.h
  - OpenTissue/math/big/big_residual.h
  - OpenTissue/math/big/big_prod.h
  - OpenTissue/math/big/big_prod_add.h
  - OpenTissue/math/big/big_prod_add_rhs.h
  - OpenTissue/math/big/big_prod_row.h
  - OpenTissue/math/big/big_prod_sub.h
  - OpenTissue/math/big/big_prod_sub_rhs.h

* BIG also have a few higher-level functionality that often is convenient. This functionality can be located in the header-files:
  - OpenTissue/math/big/big_shur_system.h
  - OpenTissue/math/big/big_cholesky_decompose.h
  - OpenTissue/math/big/big_identity_preconditioner.h

* Finally BIG has a vast set of different matrix solvers. These are located in the header files

  - OpenTissue/math/big/big_svd.h
  - OpenTissue/math/big/big_cholesky.h
  - OpenTissue/math/big/big_lu.h
  - OpenTissue/math/big/big_conjugate_gradient.h
  - OpenTissue/math/big/big_gmres.h
  - OpenTissue/math/big/big_forward_gauss_seidel.h
  - OpenTissue/math/big/big_backward_gauss_seidel.h
  - OpenTissue/math/big/big_symmetric_gauss_seidel.h
  - OpenTissue/math/big/big_jacobi.h

* The matrix solvers have been implemented to provide a c-like functional way of invoking the matrix solvers. Different overloaded solver-function versions exist allowing end-users to use default parameter settings or make their own specific settings. Also in most cases a functor interface is provided making it easier to pass the solver-methods to algorithms etc..

# A BIG Hello World Example

In the following example we will show how one can set up a linear system and solve it using a conjugate gradient solver. First one would write a include directive to define the ublas matrix types

    #include <OpenTissue/math/big/big_types.h>

Following this one would write the include header that defines the conjugate gradient solver

    #include <OpenTissue/math/big/big_conjugate_gradient.h>

Next one may define the Boost uBLAS types that are needed. This could be written like this

    typedef ublas::compressed_matrix<double> matrix_type;
    typedef ublas::vector<double>            vector_type;
    typedef vector_type::size_type           size_type;

Observe that we used the convenience name space ublas that is defined for us in the big_types.h-file. Now we can declare some data for our problem at hand

    size_type N = 10;
    matrix_type A;
    A.resize(N,N,false);
    vector_type x;
    x.resize(N,false);
    vector_type b;
    b.resize(N,false);

Next we fill-in data into A and b, and initialize the solution vector x

    for(size_type i=0;i<N;++i)
    {
      b = ?;
      for(size_type j=0;j<N;++j)
      {
        A(i,j) = ?;
      }
    }
    x.clear();

The conjugate gradient method could be warm-started with a preciously found solution. In which case x would have been assigned to the value that we want to warm start with. In this example it does not make sense to warm-start so we simply clear the x-vector. Now we are ready to invoke the conjugate gradient solver to find a solution

    double epsilon = 10e-4;
    size_t max_iterations = 5;
    size_t iterations;

    OpenTissue::math::big::conjugate_gradient(
                                              A
                                              , x
                                              , b
                                              , max_iterations
                                              , epsilon
                                              , iterations
                                              );

Observe that we choice to invoke the conjugate gradient function such that we could specify the tolerance for the convergence test and the maximum iteration count. In a real-life application one would tune epsilon and max_iterations to get the right balance between accuracy, efficiency and performance. When the function returns we can test if we have converged

    if(iterations<max_iterations)
    {
      use namespace OpenTissue::math::big;
      std::cout << "success x =" << x << std::endl;
    }

Notice that we have chosen to write our the solution vector in case we converged. There is some tricky to get this to work. The Matlab streaming operators are placed in the big-name space. Thus if we want to use these output-routines we must make sure the compiler can see them. That is we must pull in the big-name space.

This is all that is required to invoke a BIG matrix solver method and write out the result in Matlab notation.
