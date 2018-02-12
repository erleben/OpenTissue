= Design Patterns =

== Strategy Design Pattern ==

Strive towards a clean separation between data-structures and
algorithms. In principle it means that one should be able to
change the algorithms working on data stored in ones data
structure.

Here is a small example illustrating the idea. Say we want to
create a particle system. The particle system needs a container
of particles. So first we create a ``data'' class called
particle. The particle class is nothing more than a place holder
of physical properties. It may look something like this:

<pre>
template <typename math_types_>
class Particle
{
public:

typedef          math_types_               math_types;
typedef typename math_types::value_traits  value_traits;
typedef typename math_types::real_type     real_type;
typedef typename math_types::vector3_type  vector3_type;

public:

vector3_type m_position;  ///< The position of the particle.
vector3_type m_velocity;  ///< The velocity of the particle.
vector3_type m_force;     ///< The total force applied to the particle.
real_type    m_mass;      ///< The total mass of the particle.

public:

Particle()
: m_position(value_traits::zero(),value_traits::zero(),value_traits::zero())
, m_velocity(value_traits::zero(),value_traits::zero(),value_traits::zero())
, m_force(value_traits::zero(),value_traits::zero(),value_traits::zero())
, m_mass(value_traits::one())
{}

};
</pre>

Notice that the particle class expects a math type binder
argument (read about the type binder design pattern elsewhere).
This way one can more easily swap the entire matrix-vector
library used in the particle class without having to change one
line of code in the particle class.

One should also notice a little implementation trick we use here
the template argument is typedef inside the class. This is in
order to better support multiple platforms (read more in the
section about template parameter naming).

A particle system is not much fun if we only have a single
particle. So next we need a particle container class.

<pre>
template <typename P>
class ParticleContainer
{
public:

typedef P            particle_type;
typedef std::list<P> particle_container;

typedef typename particle_container::iterator       particle_iterator;
typedef typename particle_container::const_iterator const_particle_iterator;

public:

particle_container m_particles;  // All the particles in the system

public:

particle_iterator begin(){ return m_particles.begin(); }
particle_iterator end()  { return m_particles.end();   }
const_particle_iterator begin() const { return m_particles.begin(); }
const_particle_iterator end()   const { return m_particles.end();   }

public:

ParticleContainer()
{}

};
</pre>

Two important choices were made here. The particle type is given
as a template argument. This will make it more easy to create a
new particle class in the future if the need suddenly arises.
Secondly a simple standard template library container, std::list,
was used. To support iterators we simply delegated the iterators
from the std::list implementation.

Now we have finished implementing the data structures, next we can
focus on the algorithms. We need some sort of simulator that can
work on the data in order to compute the trajectories of the
particles as time progresses.

<pre>
template<typename particle_container, typename T >
inline void simulate( particle_container & particles, T const & time_step )
{
typedef typename particle_container::particle_type     particle_type;
typedef typename particle_type::math_types             math_types;
typedef typename particle_container::particle_iterator particle_iterator;
typedef typename math_types::real_type                 real_type;
typedef typename math_types::value_traits              value_traits;


real_type const dt   = boost::numeric_cast<real_type>(time_step);
real_type const dtt2 = dt*dt/value_traits::two();

particle_iterator p   = particles.begin();
particle_iterator end = particles.end();
for(;p!=end;++p)
{
p->position += p->velocity*dt + (p->m_force/p->m_mass)*dtt2;
}
};
</pre>

This is one way of making the data structure completely unaware
of the algorithms. In this simplistic examples that were created
only for illustration purpose (not for efficiency or cleverness)
one can easily extend with new algorithms simply by implementing
more free template functions that work on data in a particle
container. Of course one could also have implemented the
algorithm in other ways.

The important thing to notice is that we have tried to make a
clean break between ``storage of data'' and ``working on data''.

==Back-door Gateway Design Pattern==

<b>The problem:</b> We want to prevent end users from changing sensitive data in
our data structure, and at the same time we want to allow others to develop new
operations working on the sensitive data in our data structure.

<b>Example:</b> Imagine end users are allowed to read data, but not allowed to
write data. This would be implemented as:
<pre>
class MyInt
{
private:
int m_value;
public:
MyInt(void):m_value(2){};
public:
int get_value(void) {  return m_value; };
};
</pre>

To implement a functor that should have write access one would have to make it a
friend, that is
<pre>
class MyInt
{
public:
friend class IntTweakifierFunctor
...
};

class IntTweakifierFunctor
{
public:
void operator()(MyInt & i)
{
i.m_value = i.get_value()+1)*3;
};
};
</pre>

The problem lies in that either one must anticipate all possible names for
future functors and make these friends, or users who are creating new functors
should add their own functors as friends.

We want a design pattern, allowing users to develop new functors working on
sensitive data without having to modify the original data structure and at the
same time we want to protect sensitive data from end users.

<b>Solution:</b> The solution is to introduce a core_access class. This class
exposes sensitive data to those users implementing functors. Here is how to
implement it:
<pre>
class int_core_access
{
public:
template < typename Int,typename value >
static void set_value(Int i,value v)
{
i.set_Value(v);
};
};

class MyInt
{
private:
int m_value;
public:
int get_value(void) {  return m_value; };
private:
friend class int_core_access;
void set_Value(int value) {  m_value = value; };
};

class IntTweakifierFunctor
{
public:
void operator()(MyInt & i)
{
int_core_access::set_value(i, ((i.get_value()+1)*3)  );
};
};
</pre>

<b>Observe:</b> IntTweakifierFunctor is not a friend of MyInt, MyInt do not
expose the writing access to end users, but int_core_access is a friend of
MyInt. Finally int_core_access have a static template method matching the
private writing-method of MyInt.

<b>Benefits:</b> Data structure does not have to be modified when people
create new operations on it. The developer of the data structure can clearly
design what is sensitive data, what data is allowed to be exposed to functors
developers, and what data is prohibited from end users.

Example program <a href="main.cpp">main.cpp</a>

This design pattern was inspired by [http://www.boost.org/libs/iterator/doc/index.html The Boost Iterator Library].

== Typebinder Design Pattern ==

=== Part 1 ===
<b>The problem:</b> We want template classes to know about each other without
making their types explicit.

<b>Example:</b> Let us study a template graph data structure. It
consists of nodes having pointers to edges and edges having pointer to incident
nodes. We want to write something like:
<pre>
template< typename node_type >
class Edge
{
public:
node_type * m_A;
node_type * m_B;
};

template< typename edge_type >
class Node
{
public:
std::list<edge_type *> m_edges;
};

Node<Edge> * A = new...
Edge<Node> * E = new...
E->m_A = A;
</pre>
However, this will not work due to the cyclic dependence of the template types.

<b>Solution:</b>
To circumvent this problem we use a type binder design pattern. That is we create
a type binder struct, as follows
<pre>
template<
template< typename > class node_class,
template< typename > class edge_class
>
struct GraphTypes
{
typedef GraphTypes<node_class,edge_class> types;

typedef edge_class<types> edge_type;
typedef node_class<types> node_type;
};
</pre>
Next we re-implement the node and edge classes as follows
<pre>
template< typename types >
class Edge
{
public:
types::node_type * m_A;
types::node_type * m_B;
};

template< typename types >
class Node
{
public:
std::list<types::edge_type *> m_edges;
};
</pre>
When using it we would write
<pre>
typedef GraphTypes<Node,Edge>  graph_types;
graph_type::node_type * A = new....
graph_type::edge_type * E = new....
E->m_A = A;
..
</pre>
This design pattern was inspired by discussions with former master student <a
href="nowhere">Jonas Meyer</a>.

=== Part 2 ===
<b>The problem:</b> Continuing the graph example, the main idea, is that edge
and node should have traits specified by algorithms (i.e. policies) working on
them This should work even if one swaps an algorithm, which needs completely
different traits.

<b>Solution:</b> The Solution to the problem is to introduce an auxiliary
template class, which can bind all types together. Thus we call it a TypeBinder

The Node and the edge class and any algorithm class' take one template type,
named Types, which is supposed to be an ``instantiation'' of the TypeBinder.
This allows any of these classes to use all types defined in the type binder.

Furthermore every algorithm class must implement an inner node and edge traits
class. The TypeBinder assumes this, and will concatenate all traits into one
huge traits class.
<pre>
template<typename types>
class Edge : public types::edge_traits
{
};

template<typename types>
class Node : public types::node_traits
{
};

template<typename types>
class Algo1
{
public:
class node__traits
{
public:
typename types::edge_type * m_e1;
};
class edge_traits
{
public:
typename types::node_type * m_n1;
};
};

template<typename types>
class Algo2
{
public:
class node_traits
{
public:
typename types::edge_type * m_e2;
};
class edge_traits
{
public:
typename types::node_type * m_n2;
};
};

template<
template< typename > class node_class,
template< typename > class edge_class,
template< typename > class algo1_class,
template< typename > class algo2_class
>
class TypeBinder
{
public:

typedef TypeBinder<node_class,edge_class,algo1_class,algo2_class> types;
typedef node_class<types> node_type;
typedef edge_class<types> edge_type;
typedef Algo1<types> algo1_type;
typedef Algo2<types> algo2_type;
class NodeTraitsClass : public algo1_type::node_traits,public algo2_type::node_traits
{
};
class EdgeTraitsClass : public algo1_type::edge_traits,public algo2_type::edge_traits
{
};
typedef EdgeTraitsClass edge_traits;
typedef NodeTraitsClass node_traits;
};

typedef TypeBinder<Node,Edge,Algo1,Algo2>::edge_type edge_type;
typedef TypeBinder<Node,Edge,Algo1,Algo2>::node_type node_type;

void test(void)
{
node_type node;
edge_type edge;
node.m_e1 = &edge;

node.m_e2 = &edge;
edge.m_n1 = &node;
edge.m_n2 = &node;
};
</pre>

== Static Member initialized in Header (SMIIH) Design Pattern ==

OpenTissue is a header-only library and as such it is not very problematic to write something like

<pre>
class SomeClass
{
public:
static int const N = 0;
};
</pre>

In this case the compiler knows how to initialize the static member (the const part is not important for the discussion here). The reason why this work is because the N-variable is an integral type. Say we have something like

<pre>
class SomeOtherClass
{
public:
static double const N;
static SomeClass M;
};
</pre>

Here we get into trouble. The compiler do not know how to initialize the static members. We can NOT write

<pre>
class SomeOtherClass
{
public:
static double const N = 0.0;
static SomeClass M = SomeClass();
};
</pre>

The only way to do this is to initialize the members in a source-file. However OpenTissue is header only and this is therefore not an option. Instead we will gain the same functionality by re-writing SomeOtherClass as follows:

<pre>
class SomeOtherClass
{
public:
static double const & N()
{
static double const value = 0.0;
return value;
}

static SomeClass & M()
{
static SomeClass value();
return value;
}
};
</pre>

We have thus created member methods return local static members. The difference lies mainly in how you would use the constants with initialization in source-file one would reference the N-value like

<pre>
SomeOtherClass::N;
</pre>

With the new paradigm one would have to write

<pre>
SomeOtherClass::N();
</pre>


== Value Traits Design Patterns ==

As an example imagine we have some template class with some real type member that must be initialized to the value one. One may then immediately write down

<pre>
template<typename real_type>
class Fancy
{
public:

real_type m_x;

Fancy()
: m_x(1.0)
{}
};
</pre>

This works fine if the real type is double but if float is used one will get swarmed with compiler warnings. As a first solution one may then think that writing something like

<pre>
template<typename real_type>
class Fancy
{
public:

real_type m_x;

Fancy()
: m_x(static_cast<real_type>(1.0))
{}
};
</pre>

Solves the problem. Yes this will make sure that you get rid of the compiler warnings. However it is not very pretty and it really does not solve the problem that the compiler warning was telling you about. Another approach would be to use value traits instead. This could be done in several different ways I will here show two different ways one way using partial specialization of template functions and another using partial template specialization of template classes. So let us start out by writing


<pre>
template<typename real_type>
real_type one();

template<>
int          one<int>()         { return 1;    }

template<>
unsigned int one<unsigned int>(){ return 1u;   }

template<>
float        one<float>()       { return 1.0f; }

template<>
double       one<double>()      { return 1.0;  }
...many more specializations...
</pre>

Now one could write

<pre>
template<typename real_type>
class Fancy
{
public:

real_type m_x;

Fancy()
: m_x(one<real_type>())
{}
};
</pre>

We will get no compiler warnings and we have fixed the potential problem that the original warning was addressing. If there do not exist a partial specialization for the real_type that the end user decides to use he will get a compiler error and can then create his own partial specialization of the value one for this real type. The design is thus easy expendable. We could have done the same thing using template classes. For instance we could have written

<pre>
template<real_type>
class Values;

template<>
class Values<int>
{
public:
static int one(){return 1;}
};

template<>
class Values<float>
{
public:
static float one(){return 1.0f;}
};

... more specializations here ...
</pre>

Then we would have to slightly change our syntax in the Fancy class

<pre>
template<typename real_type>
class Fancy
{
public:

real_type m_x;

Fancy()
: m_x(Values<real_type>::one())
{}
};
</pre>

We have the same functionality as in the case of the template functions. The main difference in the two approaches lies in that we can parameterize the class-template approach that is we can write the Fancy class as

<pre>
template<typename real_type, typename value_traits = Values<real_type> >
class Fancy
{
public:

real_type m_x;

Fancy()
: m_x(value_traits::one())
{}
};
</pre>

Now the end-user can actually make his own value trait class and pass this along as a template parameter or he could use the default value traits. Also he can still extend the default value trait with his one types if he wishes to do so without having to modify existing source code. The drawback from the parameterized class-template approach is of course that the Fancy class gets one more template parameter and as such becomes slightly more complicated to look at. As a finally note I should show that the value traits approach also can be used if one desires to initialize a variable within some piece of code, that is one could write:

<pre>
real_type N = one<real_type>();
</pre>

== Tag Dispatching Design Pattern ==

<b>Problem :</b> In some cases the end-user should be able to use different versions of an algorithm or method.

<b>Solution :</b>Obviously simple overloading can not be used. To see this let us study a simple example. Without loss of generality say we have an algorithm for computing the norm of a matrix. According to the Strategy design pattern one should try to keep a clean separation between data and algorithm. We therefore parameterize the data of our norm algorithm. This would look something like this:

<pre>
template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A)
{
...
}
</pre>

However, there might exist other algorithms for computing the matrix norm or we may even be able to create specialized versions of our algorithm. Say for instance that the caller knows the matrix is a diagonal matrix or the caller knows that the matrix is stored in column-major format and not in row-major form.

We can not simply overload the norm function when we implement all these extra features. The compiler would not be able to distinguish between the different template functions (since all template functions would have the same name and take a single template argument). One solution could be to create different names for the different algorithm versions. That is one would implement the extensions like this

<pre>
template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm_diagonal(matrix_type const & A){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm_row_major(matrix_type const & A){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm_column_major(matrix_type const & A){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm_infinity(matrix_type const & A){  ...}

...
</pre>

Now this would work, but it may not always be what one wants. We would like the all the functions to have the same name and then let the compiler pick the correct version. Tag-dispatching can be used for this task. To implement the above example with TAG dispatching one would write something like

<pre>
struct default {};
struct diagonal {};
struct column_major {};
struct row_major {};
struct infinity {};

template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A, default const & /*tag*/ ){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A, diagonal const & /*tag*/ ){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A, column_major const & /*tag*/ ){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A, row_major const & /*tag*/ ){  ...}

template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A, infinity const & /*tag*/ ){  ...}

...
</pre>

Notice that we simply created a lot of dummy types, empty structs so to speak. We used these dummy types to overload the norm template function. Now an end-user can select the norm function he or she wants by specifying the tag-type in his call. Like this

<pre>
norm(A, diagonal() );
</pre>

That is basically it. In some cases one would like to add an mechanism for hiding the tags. For instance say one have another algorithm working on some matrix_type and using a norm-function. The trick is to implement a dispatcher function. A simple way could be like this

<pre>
template<typename matrix_type>
inline typename matrix_type::value_type norm(matrix_type const & A )
{
return norm(A, matrix_type::norm_tag_type() );
}
</pre>

In our example we simply extended the matrix data with a norm tag-type indicator.
