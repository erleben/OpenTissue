## Creating a Multibody Dynamics Simulator

Before one can start simulating one needs to bind together the types of
algorithms and methods that one wants to use in the simulator.
This process consists of three steps

<ul>
<li>Define the type of collision detection engine
<li>Define the type of stepper (a stepper is used to step forward in time)
<li>Bind together all types
</ul>

First we need to get access to all the types defined in the multibody dynamics engine. This can be done by included the single all-header-in-one file

    #include <OpenTissue/dynamics/mbd/mbd.h>

Now we have access to a bundle of types. Next we can start defining a collision detection engine

    template<typename types>
    class MyCollisionDetection
      : public OpenTissue::mbd::CollisionDetection<
        types
        , OpenTissue::mbd::SpatialHashing
        , OpenTissue::mbd::GeometryDispatcher
        , OpenTissue::mbd::SingleGroupAnalysis
      >
      {};


In this definition we have chosen a spatial hashing algorithm to deal with broad phase collision detection, we use a geometry dispatcher in place of the narrow phase module. The dispatcher needs to be initialized later on with collision handlers for specific narrow phase collision detection algorithms. We also specified a collision analyzer that merges all collision information into one single collection, named a group.

For the next step we define our stepper

<pre>
template< typename types  >
class MyStepper
: public OpenTissue::mbd::DynamicsStepper<
types
, OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy >
>
{};
</pre>

Here we have applied a traditional dynamics stepper using a projected Gauss-Seidel method for solving the underlying complementarity problem formulation. Finally we glue it all together in a type-binder, like this

<pre>
typedef OpenTissue::mbd::Types<
OpenTissue::mbd::optimized_ublas_math_policy<double>
, OpenTissue::mbd::NoSleepyPolicy
, MyStepper
, MyCollisionDetection
, OpenTissue::mbd::ExplicitFixedStepSimulator
> types;
</pre>

This took care of the preliminary steps, now we can start using the simulator types. For instance by creating a simulator object.

<pre>
typedef types::simulator_type                          simulator_type;

simulator_type  m_simulatr
</pre>

At some point before we start to simulate anything we need to bind collision handlers to our geometry dispatcher otherwise no narrow-phase collision detection is done. The Multibody dynamics engine provides a convenience tool for setting up a default geometry dispatcher using geometry and collision detection algorithms already implemented in OpenTissue.

<pre>
OpenTissue::mbd::setup_default_geometry_dispatcher(m_simulator);
</pre>


## ABC of Setting up a Scene

When setting up a scene, one needs a simulator, a configuration and a material library. The first knows how to move objects around in the world. The second describes the objects that needs to be moved and the last defines parameters for how objects interact in the world. The three components are declared like this

<pre>
typedef types::simulator_type                          simulator_type;
typedef types::configuration_type                      configuration_type;
typedef types::material_library_type                   material_library_type;

simulator_type        m_simulator;
configuration_type    m_configuration;
material_library_type m_library;
</pre>


Before we start it is important to learn one basic fact! The user has ownership of all the components of the simulator. The user is therefore responsible for memory allocation and correct initialization. So before doing anything else one would need to clear the components that make up a simulator.

<pre>
m_simulator.clear();
m_configuration.clear();
m_library.clear();
</pre>

Invoking clear will reset the state of the components to the same state they were in when they were instantiated. Next we need to add some objects to the configuration and setup some material parameters. Here we will just do something really simple to illustrate the concept.

<pre>
typedef types::body_type m_body;

m_body.set_geometry( &(...some geometry type...) );
... initialize state of body ...
m_configuration.add( &m_body );

... add other bodies, joints etc. to the configuration ...

types::material_type * default_material = m_library.default_material();
default_material->set_friction_coefficient(0.4);
default_material->normal_restitution() = 0.1;

... add more material if needed ...
</pre>

We also need to make sure that the simulator, configuration and material library know about each other. This is done as follows

<pre>
m_configuration.set_material_library(m_library);
m_simulator.init(m_configuration);
</pre>

Now one is ready to start simulating by invoking the run method on the simulator.

## Creating a new Geometry Type

In OpenTissue all collision geometry types should be inherited from the GeometryInterface interface. This is defined in the headerfile

    #include <OpenTissue/collision/collision_geometry_interface.h>

For the purpose of illustration we will here show how to create the imaginary geometry type, Bogus. A typical way to do this would look like this

<pre>
template< typename math_types_ >
class Bogus
: public GeometryInterface< math_types_ >
, public OpenTissue::utility::ClassID< Bogus<math_types_> >
{
public:

... implement whatever needs to be in Bogus ...
};
</pre>

The ClassID class is a utility class that will assist you in supporting the interface of
the GeometryInterface class. Once we have our geometry type we can start implementing our
own collision handlers.

For instance to we might want to create a collision handler for comparing two bogus geometries against each other.
<pre>
template<typename mbd_types>
struct BogusBogusHandler
{
typedef typename mbd_types::collision_info_type   collision_info_type;
typedef typename mbd_types::math_policy           math_policy;
typedef Bogus<math_policy>                        bogus_type;

static bool test(
bogus_type & bogus_A
, bogus_type & bogus_B
, collision_info_type & info
)
{
... invoke or do what ever collision detection needed! ...
}
};
</pre>
The function signature of a collision handler must always consist of three arguments, the first two needs
to be the geometry types, the third argument must be a collision info. The return value must be a boolean.
Now we can bind the new collision handler to our geometry dispatcher by simply writing:

<pre>
m_simulator.get_collision_detection()->get_narrow_phase()->bind( &BogusBogusHandler<mbd_types>::test   );
</pre>

One would need to implement collision handlers for every other type of geometry that Bogus should be
able to collide with (boxes, spheres etc..) and bind these collision handlers to the geometry dispatcher.

## Creating a new Complementarity Solver

One needs to do two things in order to create a new complementarity solver type.

<ol>
<li>Create a math policy</li>
<li>Create the solver</li>
</ol>

###  Creating the Math Policy

The math policy should be placed in this folder

<pre>
OpenTissue/dynamics/mbd/math/
</pre>

The folder includes two examples of existing math policies

<pre>
OpenTissue/dynamics/mbd/math/mbd_default_math_policy.h
OpenTissue/dynamics/mbd/math/mbd_optimized_ublas_math_policy.h
</pre>

If one just needs sparse uBLAS kind of matrix stuff then it would probably be easier to specialize the default math policy by creating an inherited class from the default math policy. Let the inherited class be extended with whatever interface one needs for the new solver type. This would most likely boils down to adding a sub-system solver method! This might be neat if one decides to swap the internal workings of the solver later on. Then simply create another derived class with the same sub-system interface but with different internal workings.

The multibody dynamics engine relies heavily on static polymorphism, so the math policy interface is completely unbounded. However, many main parts of the engine uses the math policy for basic manipulations. One will get compiler errors if one forgets a method in ones math policy. Deriving a math policy from one of the existing classes avoids this problem.


### Creating the Solver

To help assist users in implementing new solvers a virtual base class has been provided. So in order to create a new solver type one should just make a public derived class from this interface

<pre>
OpenTissue/dynamics/mbd/interfaces/mbd_ncp_solver_interface.h
</pre>

Fill in the blanks and in the end throw the final class into this folder

<pre>
OpenTissue/dynamics/mbd/solvers/
</pre>


This is basically all it takes.
