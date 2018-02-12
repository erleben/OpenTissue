OpenTissue is a meta-library. This means that OpenTissue is a collection of many smaller libraries. In your application you might want to use one of these libraries or several in combination with each other. Very much in the same way as you would use libraries like STL or Boost in an application.

Most of the libraries are what we would like to call self-contained. By this we mean that the libraries are ``header-only'' without any third-party dependencies. Thus to use OpenTissue in your application would in many cases simply imply that you need to tell your compiler the location at where it can find the OpenTissue include headers. If you are using any sub-libraries that have some dependency then your application will have the same dependency. As an example a substantial subset of the libraries are using Boost, so in these cases you would need this third-party dependency in your own application as well.

To be on the safe side you could add every dependency used in OpenTissue as a dependency to your own application. However, this may be overkill. If for instance you do not use any of the Cg or OpenGL utilities then there is no need to  ``link'' with these libraries. The dependency table on the [[Installing_OpenTissue|OpenTissue installation page]]. Provides an overview of all the dependencies in OpenTissue and may assist one in tracking down what dependencies are needed.


= Using OpenTissue as Third-Party Software =

OpenTissue is using CMake for generating makefiles to build OpenTissue with. This has major benefits for OpenTissue users that use CMake for developing their own applications.

There are three simple steps

# Download OpenTissue
# Run CMake on OpenTissue (See [[Using_CMake|here]] for details)
# Write you own CMakeLists.txt file

In the following we will focus on the third step, since the first two steps described elsewhere on this Wiki. The second step above will create a file

<center>
OpenTissueConfig.cmake
</center>

In the location on the hard-drive where you put OpenTissue. This file can be used by CMake to locate all the variables/settings needed by OpenTissue.

In order to see how we will take you on a step-by-step setup of an imaginary application using OpenTissue as a third-party tool. Without loss of generality let us assume that you have decided upon a source tree looking like this

<pre>

MyProject
|
+-- src

</pre>

In this case you would want to put a CMakeLists.txt file into your ``MyProject'' folder. The file would look something like this


<pre>
PROJECT(MyProject)

... Add your own Cmake specific stuff here ...

FIND_PACKAGE(OpenTissue)
IF(OPENTISSUE_FOUND)
INCLUDE_DIRECTORIES( ${OPENTISSUE_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${OPENTISSUE_LIBRARY_DIRS} )
ADD_DEFINITIONS( ${OPENTISSUE_FLAGS} )
ENDIF(OPENTISSUE_FOUND)

... Add your own CMake specific stuff here ...

SUBDIRS(src)
</pre>

The thing to notice here is the command FIND_PACKAGE(OpenTissue). When CMake sees this command it try to look for a file named OpenTissueConfig.cmake. During the first CMake run this will fail because your application do not know anything about where you decided to put OpenTissue on your hard-drive. CMake will therefore prompt you for information about where to find the file OpenTissueConfig.cmake.

When you re-run CMake after having entered the location of the OpenTissueConfig.cmake file then CMake will read the content of the OpenTissueConfig.cmake file and add appropriate cache values to CMake.

The added cache values are mainly supposed to be used to make OpenTissue find the third-party software that it needs.
Below you can find a table with the names of the added cache values and an explanation of their intended usage.

<table border="1">
<tr>
<td>Name</td><td>Usage</td>
</tr>
<tr>
<td>
OPENTISSUE_FOUND
</td>
<td>
System has OpenTissue and it can be used.
</td>
</tr>
<tr>
<td>
OPENTISSUE_INCLUDE_DIRS
</td>
<td>
Directories need to find header-files used by OpenTissue and OpenTissue itself
</td>
</tr>
<tr>
<td>
OPENTISSUE_LIBRARY_DIRS
</td>
<td>
Directories where the compiler can find static libraries of third-party software used by OpenTissue.
</td>
</tr>
<tr>
<td>
OPENTISSUE_LIBS
</td>
<td>
Full paths to all libraries of third-party software used by OpenTissue.
</td>
</tr>
<tr>
<td>
OPENTISSUE_FLAGS
</td>
<td>
OpenTissue compiler flags that should be used for any application using OpenTissue.
</td>
</tr>
</table>

Our example CMakeLists.txt file shows a typical example for the added cache values. The locations where to look for headers and libs are extended with OpenTissue specific locations and any OpenTissue specific compiler flags are also added to the project.

The next step in your project is to create a target for your application. In our simple imaginary application one might simply add a CMakeLists.txt file to the src-folder. This CMakeLists.txt file could look something like this

<pre>
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(my_target some.cpp ... another.cpp)

TARGET_LINK_LIBRARIES(my_target ${OPENTISSUE_LIBS})

INSTALL(
TARGETS my_target
RUNTIME DESTINATION  bin
)
</pre>

Most of the commands above are pretty much standard CMake things to do. The important one for making OpenTissue work with your target is the command TARGET_LINK_LIBRARIES. This command basically tells CMake explicitly what libs are needed by OpenTissue and therefore also needed to be linked with your application.

That's it, now you have created and application that uses OpenTissue as a third-party piece of software.


== Other Settings ==

If one use CMake to generate MSVC solution files then we can recommend adding something like this

<pre>
IF (MSVC80)

ADD_DEFINITIONS(-D_SCL_SECURE_NO_DEPRECATE)
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

ENDIF(MSVC80)
</pre>

to ones CMakeLists.txt files. This will eliminate deprecation warnings from the MSVC compiler.
