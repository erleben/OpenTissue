= Good Practice =

== Use ?-Notation when Possible ==

Typical one have if-statements in ones code looking like this

<pre>
if( test )
a = foo;
else
a = bar;
</pre>

Such if-statements should always be re-written into ?-notation, like this:

<pre>
a = test ? foo : bar;
</pre>

The rationale behind this is to improve efficiency of the compiled code. The ?-notation often results in more efficient code for some compilers. Thus one would hint to the compiler that it can perform some optimizations.

In general the ?-notation should be applied whenever the left-hand-side is the same and the right-hand-side and test is sufficient simple. Otherwise one would simply mis-use the ?-notation and create nearly unreadable code.

== Genericity ==
Is the code generic? Could the code be modularized differently to
increase genericity?

== Scalable API ==
Does the code support a high-level API? A sort of single function
call of the library that hides all nasty template magic from end-users not
wanting to know about it. Does the new code support a low-level interface
where experts users can tweak traits and policies as they desire?

== C++/STL compliance ==
For instance if the code includes some new container it should support
iterators etc.. In such a way that the container looks and feels as a
STL component as much as possible.

== Explicit Keyword ==
Implicit type conversions are considered to be dangerous and cause
problems for performance. Always try to use the explicit keyword in
specialized constructors as
much as possible to avoid implicit type conversions.

== Typebinder Bloating ==
Typebinders are cool and should be used when possible. However try to
minimize the size of typebinders. Split large typebinders into
hierarchies of
type-binders/traits/policies. Often OpenTissue already contains
smaller typebinders that can be re-used in such hierarchies. See
BasicMathTypes and ValueTraits.

== Implement Iterators if Possible ==
Try always to make data containers mimic STL as much as possible. For
instance if possible create iterators and create members ``begin'' and
``end'' if possible.

This way we have a better chance of reusing stuff from STL and other
libraries such as Boost.

== Documentation Correctness ==
Does the documentation and the code express the same thing. i.e. does
the documentation say one thing, and the code another?

== Use of existing OT-code ==
Has parts of the code reinvented the hot water - is there parts of the
code that could be replaced with existing OT-function-calls?
This should only be done if it makes sense in the context.

The rationale here is to reuse code as much as possible.


== Use of existing third-party ==
Could parts of the code be efficiently replaced with a call to an
existing third party library (eg. boost)?

The rationale here is to reuse code as much as possible.

== Type Safety ==
Are types handled in a safe way such that if one for instance changes
double to float everything still works?

== Error Messages ==
In general no console output should ever be made from OpenTissue
library code. However std::cerr might be acceptable in very few cases
for error handling. However, it is considered better practice to use
assert or throw exceptions for error handling.

Critical errors should be handled by exceptions. A critical error is
something that makes it quite impossible for an algorithm to run. If
the error is non-critical that is algorithms/methods can run (but might
behave unexpected or do nothing) then use an assert.

== Avoid for-loops ==
As much as possible try to use iterations, std::for_each and std::fill
instead of writing for loops yourself.

The rationale is that for-loops are known to be a major source to programming
errors. Besides we believe that using iterators, for_each, fill and other constructs
like this improves the STL compliance of OpenTissue code.

== Use of new Third-Party ==
Try to minimize the usage of third-party software unless it is really
called for. Try to use things in OpenTissue if they exist or things in
third-party software
already used in OpenTissue. It is considered a really bad thing to
introduce new third-party dependencies. If you introduce new
third-party dependencies make sure the new dependency
comply with the license of OpenTissue. Make sure they are
cross-platform, make sure they are ready for UTF8

== Use Argument Dependent Lookup (ADL) ==

ADL is used by the compiler to determine what version of a template function that one intend to call inside ones code. Templates can pretty fast get quite nasty and if not done carefully one might end up having multiple template functions all with the same name but with very different semantics.

It is therefore important when one implement library code to ensure that the compiler will be able to do ADL correctly later on when others use ones library code. In the example below we will walk you through an example of create a new Number class library having it owns max and min functions. The same techniques are applicable to any non-member template function that is part of ones library. One should use the implementation technique below to aid the compiler in doing ADL if the name of a free-function is quite common and likely to have a name clash with functions in other libraries

=== The max-min Study Case ===

First of all if you are running on windows and you are using min and max then you need to turn of a few macros that otherwise will be defined behind your back. For instance in case your need the windows include header write as follows:

<pre>
#ifdef WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#  include <windows.h>
#  undef WIN32_LEAN_AND_MEAN
#  undef NOMINMAX
#endif
</pre>

This way all the windows stuff is kept to a minimum. Now as an example say we need to implement a new data type named Number that is

<pre>
template<typename value_type_>
class Number
{
...
};
</pre>

And we want to be able to have a min function such that we can write

<pre>
Number<some_type> a,b,c;
... set values of a and b ...
c = min(a,b);
</pre>

The first thing we need to do is to make a min function, we will create it as a non-member friend function, that is we write

<pre>
template<typename value_type_>
class Number
{
...

friend Number min( Number const & a, Number const & b)
{
....
}
};
</pre>

Making the min-function a friend function means that whenever we use the Number data type the compiler pulls in the min-function. The next thing we must pay attention to is how the constructors are made for our new data type. Suppose that the default and the copy constructors are written as follows

<pre>
template<typename value_type_>
class Number
{

public:

Number(){...}

template<typename value_type2_>
Number( Number<value_type2 const & other){ ...}

...

friend Number min( Number const & a, Number const & b)
{
....
}
};
</pre>

writing the copy constructor this way may look appealing because then we can get implicit type conversions for nearly free. That is we could write

<pre>
Number<float> a;
Number<double> b;
b = a;
</pre>

However it causes a lot of problems too. For instance say we have written

<pre>
Number<double> a,b,c;
c = min(a,b);
</pre>

Now the compiler gets into a mess. It could use the double-version of the min function, but since it is allowed to do implicit type conversion it could also use the float-version of the min-function and all other types that it knows to implicitly convert to. The lesson to be learned it not to use a parameterized copy-constructor. Keep ``explicit'' control over your types and write

<pre>
template<typename value_type_>
class Number
{

public:

Number(){...}

Number( Number const & other){ ...}

...

friend Number min( Number const & a, Number const & b)
{
....
}
};
</pre>

The final thing one should remember is to pull in the standard template library's min function in case one writes generic source code. Say that we in some other header-file create the template function like listed below:

<pre>
template< value_type>
void do_something_function(value_type const & a,value_type const & b)
{
using std::min;

value_type c;
....
c = min(a,b);
}
</pre>

The using std::min makes sure that the compiler can fall back to simple types such as float and double or whatever. So to sum it up, to make ADL work you need

* non-member friend function
* no implicit type conversion on your copy-constructor
* add using std::XXX in template functions where your XXX function is used, if XXX is in std name space.

You will often need to do the above for min, max and fabs functions. The pattern is general and can be used for your home brewed functions too.

== Do Unit Testing ==

It is our policy that all library code should have unit-tests. The reasons for this decision are

* It is easier for us to catch problems when cross compiling
* Far to many times in the past un-covered code suddenly causes compiler errors. Thus we would like to make sure that as much as possible of the code is covered when we compile ``everything'' ( that is all demo applications and all unit-tests)
* Unit-tests is not any better than the person writing them, but they are a quality measure and a statement to the world that we care about creating high-quality code in OpenTissue
* With unit testing we can provide regression testing

For guidelines on how to write unit tests in OpenTissue see here

http://www.opentissue.org/wikitissue/index.php/Unit_testing

We decided upon Boost Unit testing for the following reasons:

* We already got a dependency on boost, so it seems rather stupid to pull in a new dependency for unit testing.
* With Boost 1_34_0 it is rather easy to setup unit tests. I made my own first example within a few minutes:-)
* Boost unit tests work together with CMake (which we are also using)
* Documentation is good and there is a lot of examples out in the world using boost unit testing.
* The same tool and nearly the same approach is used in www.3Dot.dk (A sister project of OpenTisuse), so from an administration viewpoint, this will make it easier for us to run OpenTissue.

When unit-testing ones library code then one should ensure:

* Type instantiations are verified. If ones library is supposed to take a real_type argument then one should try instantiating components of ones library with both float and double types. In order to make sure that it is possible to change the type of the real_type argument.
* Instantiations and simple functionality. One should not write unit-tests that tests the correctness of computations from a full-blown system nor should one write unit-tests that tests every little for-loop in your code. However, you should make sure that methods can be invoked when called with prober data. One should verify that exceptions like invalid_argument are thrown when expected etc..

The purpose of these types of unit-test is simply to make sure that all ones code compiles correctly. The main goal is coverage of the code. The above type of unit-test should be supplemented with simple correctness testing of smaller components.

For instance if one have an add function in ones library then one should have at least one unit-test testing that adding is correct. Black-box testing of correctness of smaller components is desirable.

== No Data Files ==

Keep data files in OpenTissue at a bare minimum. We like OpenTissue to be self-contained to make it easier for new users to run the demos without having to install anything else than OpenTissue. However, we also would like to keep the size of OpenTissue down. Further we would like to avoid any legal problems about copyright on data. Finally, we also do not which to maintain data files when libraries in OpenTissue change. Thus one should have very good arguments for adding new data files to OpenTissue.

It is however easier to add data files to DataTissue, but library code and demos should be able to run if end-users do not wish to have DataTissue.

== Document The Code ==

Please try to always document the code, by convention we use doxygen style (www.doxygen.org) throughout all our code. We stride towards documenting code on class level, method and member level as well. For method this should include full description of all arguments and return value. If part of algorithm intended caller/calleee relationship would be nice to have in the long description of the method.

Some people argue that this type of documentation is useless. However, we use it for automatically generating API reference manuals. We also believe it to be an important incentive to think about the code that one is writing. Thus implicitly in some cases in can increase code quality.




== No Implicit Type Conversion ==

Implicit type conversion can be very convenient but also extremely misleading and a performance drain. Implicit type convention can also make it impossible to do argument dependent lookup correctly

http://www.opentissue.org/wikitissue/index.php/Code_standards#Argument_Dependent_Lookup_.28ADL.29

Implicit type conversion can in some cases be implemented like this

<pre>
template< typename T>
class Number
{
public:

T m_value;

public:

Number(){}

templaet<typename T2>
Number(T2 const & v)
: m_value( v )
{}

};
</pre>

Thus template parameterized copy-constructors should never be allowed.


As another example consider this class

<pre>
class Number2
{
public:

double m_value;

public:

Number(){}

Number(double const & v)
: m_value( v )
{}

};
</pre>

Again we run into problems, since the compiler is capable of implicitly convert float and int types into double types. Here we must use the explicit keyword to tell the compiler not to accept any implicit type conversions. Only true double-types should be accepted by the specialized constructor. The code should look something like this

<pre>
class Number2
{
public:

double m_value;

public:

Number(){}

explicit Number(double const & v)
: m_value( v )
{}

};
</pre>

== Use CMake ==

OpenTissue has decided to use a cross-platform make tool. We have decided to use CMake.

* CMake has a large active community
* CMake is quite well documented
* CMake supports all the platforms we want to target (foremost windows and linux)
* CMake is actually really easy to use, read here http://www.opentissue.org/wikitissue/index.php/Using_CMake

For more info about CMake read here

http://www.cmake.org/
http://www.cmake.org/Wiki/CMake
http://www.cmake.org/Wiki/CMake_FAQ
http://www.cmake.org/Bug/
http://www.cmake.org/pipermail/cmake/

This means that when one develops library code one must write corresponding CMakeLists.txt files for both unit-tests and demo applications.



== Use Pre-Increment ==

Increment is often used when writing for-loops, a simple example is this

<pre>
template<typename point_container>
void do_iterate(point_container const & points)
{
typedef typename point_container::const_iterator  point_iterator;
typedef typename point_container::value_type      point_type;

for( point_iterator p = points.begin(); p != points.end(); ++p)
{
point_type q = *p;
...
}
}
</pre>

We encourage usage of ++p (pre-increment) rather than p++ (post-increment). Performance is the major reason for this. In order to do a post increment a temporary value must be created. In a sense a post increment corresponds to writing something like

<pre>
j = i;
i = j + 1;
</pre>

wheres the pre-increment should work in-place.

== No Name Prefixing ==

Name prefixing is often use to hint the caller about what he is using. For instance the prefix ``gl'' is used for all function names in the OpenGL graphics library to indicate that one is calling a OpenGL function.

However, In general there are some problems concerning prefixing and static polymorphism. Here follows a small example from the OpenTissue mesh library.

In polymesh/utils we have an utility named polymesh_compute_face_normal. A corresponding utility exist in the trimesh library named trimesh_compute_face_normal. Here we used the prefixing of the data types to make it transparent what library the utility was part of.
Now in mesh/common/utils we have a volume integrator that needs to compute the face normals. The problem is that it needs to pick whether to use the polymesh or trimesh versions. If the two polymesh and trimesh utilities have the same name we could rely on static polymorphism to pick the right version in the common/utils library.

Therefore name prefixing should be prohibited in order to support the usage of static polymorphism. OpenTissue is supposed to be a generic programming library. Static polymorphism is a very important and often used programming technique in generic programming. Therefore we have chosen to use the convention of not allowing name prefixing.

If name prefixing is not allowed then of course one might run the risk of having name clashes in the library code. It is therefore also important that one uses name spaces (explained elsewhere on this page)


== One "Template" in One Header File ==

It is a well known fact that too much template syntax in one header file make people run away scared. It seems that a shallow and thin organization of templates in header files makes it more easy for others to gain overview of the library source code. It also helps break type-dependencies between templates.

We therefore suggest to keep one template in one header file. This design suggestion should not be taken literally, but be applied with common sense.  The main rule is to group code in logical and manageable chunks. This is of course an subjective opinion and in some cases a art-form.


== Organizing Template Code ==

An ideal goal is to make library code accessible to end-users on all levels of experience. For a generic programming library such as OpenTissue  it basically divides the world into two kind of end-users: those that template-fans and those that are not. Below we will describe a way to organize code, which in our experience have worked well to accommodate both types of users. Please keep in mind there are possible other ways of going about solving this problem.


Information hiding can be an excellent way to help non-template users getting started. Say one has implemented some algorithm inside a very template class looking like this:

<pre>

template<
typename that_policy
, typename this_policy
, typename some_trait
, typename another_trait
, typename lots_of_types
, typename more_types
class BigScaryTemplateClass
{
public:

...

void run_my_algorithm( ... ) {  ... }

...
};

</pre>

Notice that the class got many template parameter, none of them which you care to know more about on a first encounter. The first thing we do is to immediately hide this nasty-looking template guy in some appropriate name space. For instance like this

<pre>
namespace OpenTissue
{
namespace mylib
{
namespace details
{

template<
typename that_policy
, typename this_policy
, typename some_trait
, typename another_trait
, typename lots_of_types
, typename more_types
class BigScaryTemplateClass
{
public:

...

void run_my_algorithm( ... ) {  ... }

...
};

} // namespace details
} // namespace mylib
} // namespace OpenTissue
</pre>

Advanced and experienced template users can still access the bare-bones of the library and gain control over every template argument of our scary class. However, to make our algorithm more accessible to end users not too familiar with templates we create some friendly looking free template functions. The only purpose of the template functions is to hide some of the nasty template syntax from the end-users and make provide some meaningful default template arguments for the nasty template class. This may look something like this

<pre>
namespace OpenTissue
{
namespace mylib
{
namespace details
{

template<
typename that_policy
, typename this_policy
, typename some_trait
, typename another_trait
, typename lots_of_types
, typename more_types
class BigScaryTemplateClass
{
public:

...

void run_my_algorithm( ... ) {  ... }

...
};

} // namespace details


template<typename real_type>
inline void run_my_algorithm( ... )
{
typedef .... ;

details::BigScaryTemplateClass< ... > my_algorithm;
my_algorithm.run_my_algorithm( ... )

}


} // namespace mylib
} // namespace OpenTissue
</pre>

In general one would make several partial specialization of the library function: run_my_algorithm. Each specialization corresponding to some typical usage of the algorithm (perhaps simply swapping some of the template arguments used in the algorithm).


== No Default Template Arguments ==

Default template arguments causes strange bindings between ones code.

For instance say one have created some algorithm taking a policy template argument. When implementing the library a sensible policy might be something that uses some third-party dependency like Boost uBLAS or OpenGL. Due to having specified this default template argument all future uses of the library are forced to the boost uBLAS or OpenGL as dependencies even though they are using Intel MTL and Direct X in their applications.

Besides the default arguments may come from other include header files in OpenTissue, forcing and end-users compiler to trash through a lot of unnecessary header files in cases where the end users decides to use non-default template arguments.

In most cases we find it safer to create non-member template functions that instantiate the default template arguments for template classes (as explained in the "Organizing Template Code" item on this page).

== Use STL/BOOST ==

To increase code re-usability and reducing maintainability of OpenTissue code we encourage to use STL or Boost as much as possible.

However, (in contradiction with the above recommendation) we also encourage to think carefully about using STL or Boost. In some cases it is much more attractive to have a library component written entirely in native C++ without any third-party dependencies.

== Do Numeric Conversions ==

In many cases one have to deal with conversion of one data type to another. In physical simulation it is often the case of converting a double-type to a float-type or something similar.

Let us immediately abstract over this and write a imaginary illustrative example. Say we have some algorithm

<pre>
template<typename matrix_type, typename real_type>
inline typename matrix_type::value_type my_algorithm( matrix_type & a, real_type const & s)
{
typename matrix_type::value_type  value_type;
value_type b = s + a(0,0)*s;
return b;
}
</pre>

The thing to notice here is that real_type and value_type may not be the same. They might even not be implicitly convertible by the compiler. For now let us assume they are convertible (ie. they could be float, double, int etc..).

One possible way to make the compiler shut up when handling these conversions are by casting. The simple-minded cast would be like this

<pre>
value_type ss = static_cast<value_type>(s);
value_type b = ss + a(0,0)*ss;
</pre>

Or one of the other types of cast, eg. dynamic_cast. This is however not the OpenTissue way of doing it. In OpenTissue we rely completely on boost::numeric_cast for handling numerical conversions. That is the OpenTissue way to handle it would be something like this

<pre>
value_type ss = boost::numeric_cast<value_type>(s);
value_type b = ss + a(0,0)*ss;
</pre>

The reason is that boost::numeric_cast is more safe, and it performs boundary checks in debug mode.

== Use Assertions ==

In order to enhance code quality and make it more safe to invoke/use OpenTissue functionality. It is highly recommended to make sanity tests inside OpenTissue library code. Also it is highly recommended to test validity of parameter values etc..

We recommend to use assertions for this purpose. The reason for this is that people are only impacted by the performance drawback when running in debug mode.

== Use Exceptions ==

In some cases OpenTissue end-users may call OpenTissue functionality with completely insane values or using OpenTissue parts in a right-down wrong way.

If the "usage" is critical then we recommend that one considers throwing an exception instead of merely doing assertions.

== Remember Return Value Optimization ==

TBD...

== No Grand Include Headers ==

To ease compiler load (and pre-pare for precompiled headers) it is forbidden to use the grand-include headers inside OpenTissue library components. For example if one is developing a particle system that is using some mesh utility (compute face normal or something like that) then one should not write

<pre>
#include <OpenTissue/mesh/mesh.h>
</pre>

Instead one must dig out the specific header implementing the utility (data structure or whatever functionality that ones desire) and include this explicitly.

The grand-include headers are merely a convenience feature for OpenTissue end-users, so they do not need to learn all about how files are organized inside OpenTissue.

== Keep Library Folder Structure ==

To help developers in keeping the strategy design pattern we have decided upon using the same look-and-feel layout of all OpenTissue library folders. It also have the benefit of creating a consistency of all the libraries in OpenTissue. This will hopefully make it easier for end-users to dig their way through the code.

Here is an example of a imaginary folder structure that can help   assist one in achieving this

<pre>
$(OPENTISSUE)\OpenTissue\...\mylibary\
$(OPENTISSUE)\OpenTissue\...\mylibary\io
$(OPENTISSUE)\OpenTissue\...\mylibary\util
$(OPENTISSUE)\OpenTissue\...\mylibary\policies
</pre>

In general data structures should go into the main mylibrary folder. Also algorithms/methods working on the data structures can be located in this folder.

In general there should be a grand-include header located in the main mylibrary folder. The grand include header is convenience include header. It purpose is to make life easier for end-users that do not wish to know anything about how all the files of ones library are organized. Input/output routines should go into the io sub folder. Smaller independent functions, algorithms or possible entire sub libraries can be added to the util sub folder. Also one can use a policies sub folder to collect all policies for algorithms etc. in one place.

== Be Const Correct ==

Be const correct; This basically mean that you should write down explicitly what is const and what is not. Here is a few examples illustrating what to watch out for

<pre>
template<typename real_type>
inline real_type add( real_type const & a, real_type const & b)
{
...
}
</pre>

Here we want to make sure to tell the compiler and caller that we do not mess around with the arguments. As another example

<pre>
class MyClass
{
public:

void do_something ( ... ) const { ... }

};
</pre>

This time it is important to indicate the the method do_something actual do not do anything with anything stored inside an instance of this class.

A more non-obvious example is this

<pre>
void some_function(...)
{
...
float const K = 0.001f;
...
}
</pre>

Here a constant value is created.  The const keyword is important although the value is a temporary most likely created on the call stack by the compiler and never exposed to any one outside the function code. The reason why the const keyword is important is for performance. Some compilers are able to see this is a constant and simply substitute its value into all expressions and optimize the compiled code.

== No Drawing Members ==

OpenTissue data structures and algorithms should be cleanly separated from drawing/rendering functionality. There is no need for a geometry type to have a draw method that implements for instance openGL specific drawing. This is wrong and it adds an openGL dependency on the geometry.

Some stuff in OpenTissue (Scientific Visualization, i.e. volume visualization) is hardwired to openGL and that is fine, because here the purpose of the library is visualization. If the purpose of the library is deformation then it should not include drawing functionality in any dependent way.

It is our policy to provide demos and basic utilities for openGL debug drawing to show how to interact and extract information from the libraries in OpenTissue. This is why we have stated that OpenTissue only uses openGL, but that does not mean that it does not work with direct X (or other graphics libraries for that matter).


=== Elaborate Discussion ===

OpenTissue contains a lot of geometry types both a face-indexed array, half-edge data structure, tetrahedral mesh and much more. If a physical simulated object is tied to some kind of mesh geometry then it is possible to extract the coordinates and normals very easy, and these can be passed along to any client code responsible for the actual drawing.

OpenTissue should only be concerned with the physical simulation part. The main reason for this is that OpenTissue is a meta library and not an out-of-the box engine with a bounded interface. Rendering is in our opinion not something that belongs in a meta-library for physical based animation and simulation.

There are also another reason. For most practical usages one would use in-place geometries for rendering and not the geometries used by some ``physics'' engine. In fact we would claim that in a real application a single object would be represented by several different types of geometries:

* A computational mesh
* A collision geometry
* A rendering geometry

There might even by more kinds of geometries (like AABBs for broad-phase collision detection).

One may argue that this is just redundancies and one could do with a single geometry type. However, this is not the case. The different geometry types provide different information at different scales. As an example: for a deformable model it may not even make sense to render the computational mesh nor the collision geometry. Because the computational mesh might just be a regular grid fixed in model space, the collision geometries may consist of some BVH tree.

This raises one major problem! How does one deal with all these different types of geometries? Well in most cases there is no need to do anything. For instance in a multibody simulation one just keep track of which rendering geometries that match the rigid bodies. This can be handled without even involving the multibody simulator. For deformable models one often need a coupling between the rendering geometry and the collision geometry. This is just pain-sticking. In OpenTissue we do have some mesh-coupling tools, but as of this writing (September 2007) they only provide one-way coupling. The main technique applied boils down to a matter of interpolation. Computational fluid dynamic is in fact a little different, here the rendering geometry needs to be created in some way. Thus the result of a frame simulation would be an instance of some mesh data structure or some implicit representation of the free surface. In the later case it would be the responsibility of the ``user'' of the fluid simulator to find a way to render the free surface.

To conclude in short keeping rendering geometries in sync with other geometries is painful. Most OpenTissue demos show how different libraries in OpenTissue deals with the problem. We do provide some basic tools to assist end-users in creating and maintaining rendering geometries. In the end it is the end-users that create engines using OpenTissue that needs to put this glue into their engine, because they are the only ones knowing how their engine should be used.


== Use Small Type Binders ==



Type binders is extremely convenient when programming large libraries. One problem is however that one often ens up having too many types defined in a type binder. The consequence is that one looses oversight and possible end-users of the library code have no chance in hell to get an overview of all the types being used in a type-binder.

The type binder design pattern seem to have three main drawbacks. Firstly the number of template arguments have a tendency to become quite high making the type binder seemingly complex to look at. The template arguments often have default types which implies that if one substitutes with ones own type then one gets a lot of header file dependencies on unneeded template stuff. Secondly the number of convenience types inside the type binder template class also have a tendency to grow rapidly, thus a type binder often have too many types for an end-user to to keep an overview. Finally adding new functionality to a library based on a type binder design often implies that one have to add new types into the type binder. Thus one have to alter/modify existing source code.

In conclusion:

* The type binder design is not very shallow
* The type binder design has limited generic ability

Therefore as a general rule of thumb, try to keep type-binders small. If possible organize them into several components, possible in a hierarchical manner. An example of a good typebinder is the OpenTissue Math types type binder. It only has like 10 types all within the same narrow context.

Using something like the math types type binder as a smaller component (or template argument) in ones own type binders ease a lot of coding effort and hopefully enhances code readability and overview.
