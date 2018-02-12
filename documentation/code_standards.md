= Introduction =

On this page one can read about all the code standards that are used in the OpenTissue project. The OpenTissue Board is responsible for defining the code standards. The page is intended to aid new Developers conform with the OpenTissue code standards. The sole purpose of this page is not just to tell what the code standards are, but also to explain why the code standards are as they are. When dealing with issues such as style and layout it is often a subjective meaning what is better or more nice looking. Please keep in mind that all statements on this page is the collective opinion of the OpenTissue Board.

The element of good style. Here is a few links

#[http://www.research.att.com/~bs/bs_faq2.html  Bjarne Stroustrup's C++ Style and Technique FAQ]
#[http://www.icce.rug.nl/documents/  C++ Annotations]
#[http://www.parashift.com/c++-faq-lite/index.html and more...]

== Definitions by OpenTissue Board ==

Here we explain a few words that keeps poping up, and how we define them:

* <b>Code Standard :</b> Naming conventions, formating, documentation.
* <b>Compliance Test :</b> Compliance testing tries to determine if the code adheres to the coding standards of the software project.
* <b>Generic Code :</b> In a sense: "Once written never re-written". However this is a limited view. By generic code we also imply a certain level of type invariance, convenience and high user ability of the code. The code should therefore be easy to alter, change and modify without having to make changes to actual source.

= Formating and Naming Conventions =

== Typedef Alignment ==

In order to ensure a consists look and feel we have decided upon a specific formating for large blocks of typedefs. We believe this also enhances readability of the code.

* A typedef should appear as a one-liner, that is all the code should be written on a single line of code.
* Several lines of contiguous typedefs must be aligned in columns using white spaces.
* The keyword typedef must be one column, the keyword typename must be the second column, the type declaration the third column and finally the fourth column is supposed to be the new type name.

Here is a small code example

<pre>
typedef          int                       int_type;
typedef typename math_types::real_type     real_type;
typedef typename math_types::vector3_type  vector3_type;
typedef          char const *              const_string_type
...
</pre>


== Typedef Placement ==

In order to ensure a consists look and feel we have decided upon a specific formating for large blocks of typedefs. We believe this also enhances readability of the code.

* All typedefs should appear in the top-most position and preferably be collected in contiguous blocks of logic coherence

Here is a small example

<pre>
template<typename math_types>
class MyCalculator
{
public:

typedef typename math_types::real_type     real_type;
typedef typename math_types::vector3_type  vector3_type;
typedef typename math_types::size_type     size_type;

...
};

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

== Const Placement ==

In OpenTissue we use the convention

<pre>
int const &
</pre>

rather than

<pre>
const int &
</pre>

#[http://www.research.att.com/~bs/bs_faq2.html#constplacement Bjarne Stroustrup's saying on the matter]

However

#[http://docs.linux.cz/programming/c++/www.icce.rug.nl/documents/cplusplus/cplusplus03.html#l36 const ]
#[http://docs.linux.cz/programming/c++/www.icce.rug.nl/documents/cplusplus/cplusplus06.html#ConstFunctions const functions]
#[http://www.parashift.com/c++-faq-lite/const-correctness.html#faq-18.5 const correctness]

The reason for this convention is firstly of all to be consistent to make all OpenTissue code have the same style, look and feel. Secondly we currently believe this convention is better for readability and understanding of the code. Here is some more illusive examples

<pre>
int const *
</pre>

Reading from right to left we immediately see this is a pointer to a constant integer. Whereas

<pre>
int * const
</pre>

is a constant pointer to an integer. With our choice of convention one only have to remember that the const keyword tells something about the token immediately to the left of it.

== No Tabs ==

To ensure portability on multiple platforms and support for multiple integrated development environments no tabulator characters are allowed in any OpenTissue code.

Instead all indentation should be done using exactly two white-spaces (blanks) for each indent.

== Curly Brace Formating ==

In order to ensure a consists look and feel we have decided upon a specific formating for curly braces.

* Curly braces should in general appear as the sole character on a single line
* One can deviate from the above rule if and only if the written code conforms nicely to a one-liner, that is all code can be written in a readable manner on a single line.
* New "blocks" of code should be indented with exactly two white-space characters.

Here is some illustrative examples

<pre>
void foo()
{
if ( true )
{
...
}
else
{
...
}

for(int i=0;i<10;++i)
{
...
}
}

class Foo
{
public:

Foo()
{}

void run()
{
...
}
};
</pre>

== Initializer List Alignment ==

OpenTissue uses a specific format for writing an initializer list. The reason is to ensure consistency of code, also to have the same look-and-feel everywhere. Also we believe that it enhances clarity of the code and it is easier to edit the code.

In general each item must appear on a single line, the item separator must be the first character on the line.
There should be exactly one whitespace between the separator and the following list item. All the lines must be aligned with the separator character.

Here is a small example

<pre>
class MyClass
{
public:

int m_one;
int m_two;
int m_four;
int m_three;

MyClass()
: m_one(1)
, m_two(2)
, m_four(4)
, m_three(3)
{}
};
</pre>

Note that the items must appear in the same order as they are declared in the class. If not this will generate compiler warnings on some compilers.

== Template typename/class keyword ==

In templates we prefer the keyword ''typename'' over the keyword ''class''.

In cases where it is necessary to use template-template arguments, the keyword ''class'' will have to be used.

Example:

<pre>

template< template< typename T> class T2>
class SomeClass {
...
};

</pre>

== Template Parameter Alignment ==

OpenTissue uses a specific format for writing an list of template parameters. The reason is to ensure consistency of code, also to have the same look-and-feel everywhere. We also believe that this standard enhances clarity of the code and makes it is easier to edit the code.

In general each item must appear on a single line, the item separator must be the first character on the line.
There should be exactly one whitespace between the separator and the following list item. All the lines must be aligned with the separator character.

Here is a small example

<pre>
template <
typename first_type
, typename second_type
, typename third_type
>
class MyClass
{
public:

};
</pre>

Of course the same rule applies to template functions.

== Namespace Formating ==

In order to ensure a consists look and feel we have decided upon a specific formating for name spaces.

* The OpenTissue name space is the topmost name space and all other name spaces must be placed inside the OpenTissue name space
* The OpenTissue name space must always be written with uppercase O and T and the remaining letters in lowercase.
* All other name spaces within the OpenTissue name space must be written in lowercase letters. An underscore character may be used to separate words.
* Curly braces are written on a single line, the ending brace of a name space must be followed with a C++ comment indicating the ending name space scope
* Nested name spaces should be indented with exactly two white space characters.

Here is an example illustrating all of the above rules

<pre>
namespace OpenTissue
{
namespace math
{
}// namespace math

namespace my_funny_place
{

} namespace my_funny_place
} // namespace OpenTissue

</pre>

== Classes, Members and Function Naming Conventions ==

In order to ensure a consists look and feel we have decided upon a specific formating for name spaces.

* The first letter in each word of a class name must be written with uppercase. The remaining letters must be written in lowercase.
* If the class name consists of several words then these are simply concatenated.
* All data members within a class must have a string prefix "m_"
* All data members must be written in lower-case. If names consist of several words then these must be concatenated using an underscore
* All methods must be written in lower-case. If names consist of several words then these must be concatenated using an underscore
* All functions must be written in lower-case. If names consist of several words then these must be concatenated using an underscore

Here is a small example

<pre>
class MyClass
{
public:

int m_my_member;

public:

void my_method () {}

};

void my_function () {}
</pre>


== Include Guards and Copyright Notice Placement ==

Every library file of OpenTissue should comply with a layout like this

<pre>
#ifndef OPENTISSUE_XXX_H
#define OPENTISSUE_XXX_H
//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//

...

// OPENTISSUE_XXX_H
#endif

</pre>

Here the XXX-part should be replaced with a string indicating folder, file and name space location of the code contained in the header-file (read more about this in our section on folder and file naming conventions). In short summary one should make sure that there is a one-to-one relationship between the include guards and the file paths.

Observe that the first two lines of the file is always the include guard. This is important because some compilers only look for include guards in the first 10 lines ore so of a header file. Observe that a comment is used on the final #endif to indicate what the #endif is matching the include guard of the header file.

Furthermore notice that every header file must end with a single empty line. This is to avoid compiler warnings/errors when compiler with GCC.

The copyright notice should be written exactly as shown above.

== Use Configuration Header File ==

Every include header in the OpenTissue library must include our global configuration header file, OpenTissue/configuration.hpp, This include should be placed immediately after the copyright notice. As shown below


<pre>
#ifndef OPENTISSUE_XXX_H
#define OPENTISSUE_XXX_H
//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>
...

// OPENTISSUE_XXX_H
#endif

</pre>

The configuration file will setup pragma directives needed by specific compilers. The header file also defined any other pre-directives. Such as defines that make sure that windows min max macros are not defined inside OpenTissue library code in case OpenTissue is used on a windows platform. Other platform/compiler specific tasks may be performed in this header file.

== Template Parameter Naming  Convention ==

If template parameter names are ``spelled out'' then we use the
convention to use lower case letters for the template parameter
names. Individual words should be separated with an underscore.

Here is an example of a free template function that can add two
floating point numbers.

<pre>
template<typename real_type>
inline real_type add( real_type const & a, real_type const & b)
{
return real_type(a + b);
}
</pre>

Template parameters should have meaning full names indicating what
they are used for. In the above example the name clearly hints at
a floating point type.

A trailing string like ``_type'' in the above example should be
used to indicate what kind of template parameter one is dealing
with. In OpenTissue we often use the strings shown in the table below.

<center>
<table border=1>
<tr><td>_type</td><td> A canonical type, something that one could instantiate or use as data</td></tr>
<tr><td>_trait</td><td> Often a parameterized type traits</td></tr>
<tr><td>_types</td><td> A particular kind of type trait class known as a type binder</td></tr>
<tr><td>_policy</td><td> A policy class</td></tr>
<tr><td>_algorithm</td><td>A algorithm</td></tr>
<tr><td>_method</td><td>Similar to an algorithm</td></tr>
<tr><td>_functor</td><td>A functor like type</td></tr>
<tr><td>_container</td><td>A container like type</td></tr>
<tr><td>_iterator</td><td>A iterator like type</td></tr>
<tr><td>_tag</td><td>A tag type, used for tag dispatching</td></tr>
</table>
</center>

The collection of strings is far from completely, and it is
perfectly legal to make ones own additions. The only requirement
is that the trailing string should be information and descriptive
in order to enhance readability and understandability of the code
when the code is read by others.

== Expose Template Parameter Names  ==

If a template argument should be used elsewhere in the code then
the template argument should immediately be typedef'ed inside the
class with the ``real'' name one want to use and refer to later
on in the code. This is in order to support multiple platforms.
In some compilers one can not see the template class arguments
elsewhere. Using our little trick we circumvent this problem.

Here is an example of a class that needs a math type binder argument.

<pre>
template <typename math_types_>
class MyClass
{
public:

typedef          math_types_               math_types;

... a bunch of other stuff ...
};
</pre>

The little implementation trick ensures that descendants of the
class can know about the math types or some algorithm can know
about the math types regardless of the platform and compiler one
is using.

== Short Template Parameter Names ==

In order to enhance readability of code and not have exceedingly
long lines in compiler output. It can sometimes be convenient to
use shorter names for the template parameters.

Here is a code example

<pre>

templaet<typename T>
class Foo
{
public:

typedef T real_type;

public:

T add(T const & a, T const & b) { ... }
T sub(T const & a, T const & b) { ... }
T mul(T const & a, T const & b) { ... }
T div(T const & a, T const & b) { ... }
....

};
</pre>

Observe that the name T is used internally inside the class. Also
T is exposed with a typedef clarifying what T is supposed to be.

In an implementation strategy like this is chosen then the short
template parameter names should be written in upper-case.
Preferably only one or two characters, like T, P, V, C, or
whatever that is a nice ``short'' for the real template paramter
name.

== Folder, File and Name space Naming Conventions ==

WIP. More is to be said about this!

Definition: An OpenTissue library is a collection of header files placed within a single top-level folder and which can be used independently of every other OpenTissue source code.

In short a library is a bunch of independent header files. Some rules of conduct:

- Each library should have its own name space
- All header files in a library should be placed within a single top-level folder
- The top-level folder should have the same name as the name space.
- Data structures should be placed in header files in the top-level folder
- io routines should be placed in a sub folder named io
- utilities should be placed in a sub folder named utils

The naming convention for all library files should be

<center>
namespace_functionname.h
</center>

or

<center>
namespace_classname.h
</center>

function/class names should NOT be prefixed with any string values. Thus

* PREFIXING should be avoid at all times

We decided upon these rules for the following reasons:

1) No two files must have the same name. Some compilers may run into problems with identical file names.
2) Prefixing inhibits the ability to do static polymorphism.
3) There should be a clear connection between the filename of a header file and the content in the header file.
