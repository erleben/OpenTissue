<h1>Damped Wave Equations Programming Guide</h1>

<h2>Introduction</h2>
The Damped Wave Equations (DWE) is a simplified variant of the linearized Shallow Water Equations (SWE), which is a subject to the Computational Fluid Dynamics (CFD) library implemented in OpenTissue. The DWE system basically operates a 2D height map grid using, and employs the FDM to describe the diffusion of waves. By design DWE is de-coupled from any application relevant data.

The DWE system can be employed simply by including the following header file:
<pre>
#include &lt;OpenTissue/dynamics/cfd/swe/damped_wave_equation.h&gt;
</pre>
An accompanying demo application is located in:
<pre>
/demos/opengl/waves
</pre>
The waves demo application shows how to setup the DWE system. Two particular cases are supplied; a wave tub and a rain puddle. The demo also illustrates how to retrieve the height map information from the DWE system to render the waves.

<h2>Setting up the system</h2>

The damped wave equations take 3 mandatory template type arguments; a real number type, an index type, and a 3D vector template type respectively, e.g.:
<pre>
typedef OpenTissue::DampedWaveEquations<double, int, OpenTissue::vector3>  dwe_type;
</pre>
A 4th optional template argument can also be used to specify additional traits to the DWE grid particles. By default the empty class is used, i.e. no traits, but one can freely specify any particle traits needed, e.g.:
<pre>
class MyOwnDWEParticleTraits
{
public:
vector3_type  m_coord;
};

typedef OpenTissue::DampedWaveEquations<real,index,vector3, MyOwnDWEParticleTraits>  dwe_type;
</pre>
The DWE system must be initialized before any waves can be simulated. This is done calling the '''init()''' function:
<pre>
bool init(index_type columns, index_type rows, real_type const& default height = 0);
</pre>
The function simply requires the size of the 2D grid indicated by '''columns''' > 0 (X) and '''rows''' > 0 (Y). The 3rd optional parameter can be used to specify a default water height other than 0. This value isn't really important as one can choose to use the water grid height values relatively.

The function fails if '''columns''' < 1 or '''rows''' < 1.

A DWE system uses 3 different physical parameters to solve the wave equations. These parameters can be set, read, and modified at any time using the following functions:
<pre>
real_type& damping();
real_type const& damping() const;

real_type& speed();
real_type const& speed() const;

real_type& spacing();
real_type const& spacing() const;
</pre>
The '''damping''' and '''speed''' parameters control the behavior of the waves over time and distance. The '''spacing''' parameter sets the uniform grid step size between adjacent grid particles on both axes. The 3 parameters have default values that are set to something meaningful in the sense that they will not render to system unstable during simulation.

An additional parameter can be accessed to tell the system to either use a smooth wave solver or not:
<pre>
bool& smooth();
bool const& smooth() const;
</pre>
The '''smooth''' parameter tells the wave solver to use all 8 grid neighbors in the FDM computations, otherwise only the 4 axis neighbors are taken into account. The smooth solver generates somewhat smoother waves (you could have fooled me, doc!) but is also slightly more expensive.

By default the smoothed wave solver is enabled.

<h2>Wave simulations</h2>
To simulate the waves advance in time and space simply call
<pre>
void simulate(real_type const& dt);
</pre>
At first nothing should happen, and you'll have a calm water surface. To get things more interesting you'll need to create waves, drops, ripples, etc. implicitly by influence the grid heights explicitly. This is the beautiful part. The only thing required to start the action is to modify the particle heights, and the wave solver will do the rest.

As an example to create a tiny drop at a user grid position (i, j) with a user specified water height:
<pre>
dwe_type dwe;
...
...  // initialize DWE system, etc.
...
dwe.particle(i,j).height() -= height;
</pre>
To create more heavy drops expand the influenced area to the neighborhood around (i,j) or use larger heights. The wave length is directly dependent on the connected area of influenced grid particles, where the wave amplitude depends on the height.

Boundaries can be set up by fixing particles and thus telling the DWE system to render them unmovable. As an example to create the natural boundaries of the water surface and let anything else be active:
<pre>
dwe_type dwe;
...
...  // initialize DWE system, etc.
...
for (index_type j = 0; j < dwe.rows(); ++j)
for (index_type i = 0; i < m_dwe.columns(); ++i)
dwe.particle(i,j).fixed() = j==0 || j==dwe.rows()-1 || i==0 || i==dwe.columns()-1;
</pre>
If obstacles, e.g. rocks, etc. are present in your lake just fix the particles under the them. The solver will make sure the waves will reflect back from the obstacles in the water accordingly.

To obtain a continuous wave animation call the '''simulate()''' function in succession. The time step isn't required to be fixed, but it shouldn't vary too much either. To test your wave configuration try '''dt''' = 0.02. If your system blows up you might want to tweak the parameters or use a smaller time step, say '''dt''' = 0.01.

<h2>Rendering the waves</h2>

By design the system of damped wave equations has been completely de-coupled from any visualization. The responsibility to visualize the water surface lies within the application. In fact the DWE system has no idea about any water mesh, etc. even though some visualization information can be saved in the user defined particle traits.
However, the DWE system supports read-only functions to retrieve the grid particles, and to compute the surface normals.
<pre>
index_type size() const;
dwe_particle const& particle(index_type n) const;

dwe_particle const& particle(index_type i, index_type j) const;

vector3_type normal(index_type i, index_type j) const;
</pre>
The particles contain information on their fixed states and current heights, which can be used to deform a mesh by adding the heights to the mesh nodes, etc. The '''normal()''' function will return the surface normal based on the height difference, which somewhat is a mandatory requirement for any wave surface shader.
