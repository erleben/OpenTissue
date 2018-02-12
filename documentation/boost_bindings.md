# Easy Boost Bindings installation
Goto the location where you want to install the boost bindings, and enter the following commands

    svn co http://svn.boost.org/svn/boost/sandbox/numeric_bindings-v1/boost/numeric/bindings  boost/numeric/bindings
    svn co http://svn.boost.org/svn/boost/sandbox/numeric_bindings-v1/libs/numeric/bindings libs/numeric/bindings

When you run CMake specify the BOOST_BINDING_PATH cache variable if CMake can not detected the setting itself.

That is it!

# BAT file for Windows
For windows users OpenTissue provides a bat-file
  - third_party/include/install_boost_bindings.bat

One can use this to install Boost Bindings into a default location in the OpenTissue third party folder structure.

# The Atlas Path Problem
Also, you compiler might complain that the includes cblas.h and clapack.h cannot be found. In this case, edit the files
  - boost/numeric/bindings/atlas/cblas_inc.h
  - boost/numeric/bindings/atlas/clapack_inc.h

such that the lines

    #include <cblas.h>
    #include <clapack.h>

become

    #include <atlas/cblas.h>
    #include <atlas/clapack.h>

See footnote [2.1] in libs/numeric/bindings/atlas/doc/index.html for more information.
