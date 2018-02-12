= Utility Programming Guide =

== Utilities ==

OpenTissue contains a variety of small tools that are not directly related to the physics-based animation. All these tools are located in the subfolder

OpenTissue/utility/

Further, some of these tools have become large enough to form a sub-library of their own and have therefore been placed in the own subfolder inside the utility subfolder. These sub-libraries are surveyed below. Here we will briefly look at some of the most usefull utilities

----

A small utility function has been implemented, in order to make it easy to interact with the system environment variables, regardless of whether one runs linux or windows. The function is located in the header file:

#include<OpenTissue/utility/get_environment_variable.h><br />

One can use this function by including the header file and the calling the function as follows

#include<OpenTissue/utility/get_environment_variable.h><br />std::string path = get_environment_variable("OPENTISSUE");<br />

This will retrieve the string-value of the environment variable OPENTISSUE. Which (if one have installed OpenTissue correctly) points to the folder, where OpenTissue is installed on your harddrive.

----

A set of memory query functions for windows applications have also been added to OpenTissue. These memory query functions are located in the header file:

#include<OpenTissue/utility/get_system_memory_info.h><br />

----

Working with STL hash_map is not always pleasant due to compiler differences. Therefore OpenTissue contains a header wrapper:

#include<OpenTissue/utility/hash_map.h><br />

This header takes care of differences between various compilers implementation of the hash_map container.

----

Calling the QuickHull library from inside a function can be a little difficult due to difference in calling conventions between C and C++. OpenTissue has a header file that cleanly includes the QuickHull library.

#include<OpenTissue/utility/qhull.h><br />

If one wants to use QuickHull in OpenTissue, then all one has to do is to include this header file.

----

Timing computations is often needed. OpenTissue has a high resolution timer implemented for this purpose.

#include<OpenTissue/utility/high_res_timer.h><br />

It is a template parameterized class taking one argument. The argument indicates the data type used for internal computations. We recommend using double or single precision floating points. Example usage:

HighResTimer<double> timer;<br /><br />  timer.start()<br />  ...<br />  timer.stop()<br />  std::cout << "It took " << timer() << " seconds to do it" << std::endl;<br />

----

When setting up configurations for animation or simulation, it is often quite useful to have a unique way to identify objects. OpenTissue has an identifier utility for this purpose. The utility is located in the header file:

#include<OpenTissue/utility/identifier.h><br />

It is used by inheriting from this class, as e.g.

class MyObj : public Identifier<br />{<br />...<br />};<br />

When objects are instantiated, they get a default identifier consisting of a string value and an unique index number. The string value can be changed as one pleases. Both the index and string value are usefull for storing objects in lookup tables such as hash_maps.

----

A priority heap is often needed with the following properties:

# <nowiki> Each priority value is coupled to some feature/element from another data structure. As an example, one may store edges of a polygonal mesh in a priority queue. The priorites indicate the benefit of performing some operation on the edges. Thus when extracting the top element, one gets the ``edge'', which will result in the largest gain when performing some operation on it. </nowiki>
# <nowiki> After having performed an operation on a coupled feature, one often need to recompute the priority value, and the operation may even affects ``neighboring'' features, which also need to get their priority values updated. Thus we are often faced with the problem of changing only a small number of heap values, destroying the heap-property. Since the number of heap elements is low it would be more attractive to have a heapify functionality that exploits this fact. </nowiki>

OpenTissue implements a Heap class that makes it easy to deal with the priority-feature coupling and to be able to heapify single elements rather than the entire heap. The Heap class is located in the header file:

#include<OpenTissue/utility/heap.h><br />

For example usage, see the implementation of the polymesh_triangulate function located in the header file

#include<OpenTissue/mesh/polymesh/util/polymesh_triangulate.h><br />

----

Keeping iterators to elements can sometimes be a problem when using a data container such as the STL vector. The problem is that the iterators are not persistent on insertion and deletion. This is unfortunate, therefore OpenTissue provides an alternative: an index based iterator, which is implemented in the header file

#include<OpenTissue/utility/index_iterator.h><br />

----

One may desire to iterate over data stored in a STL map container. When doing so, it can become tedious that elements of such a container really is a pair. For iterating over the data only, one only cares about the second member of the pairs. OpenTissue therefore has a map data iterator.

#include<OpenTissue/utility/map_data_iterator.h><br />

Example usage is given below:

std::map< int, char > A;<br />  ... add some data...<br />  map_data_iterator begin( A.begin() );<br />  map_data_iterator end( A.end() );<br />  map_data_iterator a = begin;<br />  for(;a!=end;++a)<br />    *a = 'a';<br />

OpenTissue also contains a map indirect data iterator. It is located in the header file:

#include<OpenTissue/utility/map_indirect_data_iterator.h><br />

This iterator is similar to the other, but is very convenient, when the stored data is pointers to something else. In such a case one often wants to be able to avoid having to write a double derefering. An example of its use is:

std::map< char, int * > A;<br />  ... add some data...<br />  map_data_iterator begin( A.begin() );<br />  map_data_iterator end( A.end() );<br />  map_data_iterator a = begin;<br />  for(;a!=end;++a)<br />    *a = 2;<br />

----

== Boost ==

For compiler reasons OpenTissue contains a subset of boost wrappers. These insulate some of the most commonly used boost header files from stupid compiler warnings. The boost header file wrappers are placed in the folder

OpenTissue/utility/boost/

We refer the reader to files in this folder for more details.

----

== OpenGL ==

OpenTissue contains in-built support for OpenGL drawing, all OpenGL related headers are placed in the subfolder

OpenTissue/utility/GL/

The OpenGL support is quite extensive an a good place to start is the [[OPENGL|The OpenGL Programming Guide]]. Note, one is not required to use OpenGL with OpenTissue.

----

== Cg ==

The builtin OpenGL support in OpenTissue has been integrated with a Cg (C for graphics) programming interface. The Cg interface makes it easy to implement shaders and to use the GPU for general purpose processing. The Cg interface is located in the subfolder

OpenTissue/utility/Cg/

Shader programming using the OpenTissue Cg interface is explained in more detail in the [[SHADER|The Shader Programming Guide]]. It would be worth to read the [[OPENGL|The OpenGL Programming Guide]] and the [[TEXTURE|The Image and Texture Programming Guide]]
