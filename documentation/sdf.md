<h1>Signed Distance Field Collision Library</h1>

The signed distance field (sdf) collision library is based on a double
representation of the object geometries. To use this library one would need to
include the header file

    #include <OpenTissue/collision/signed_distance_field/sdf.h>

This header file will include all other parts of this collision library and
hence forth one can use all the functionality in the library.

This library has an accompanied demo application located in

<pre>
demos/opengl/sdf_collision
</pre>

This demo application shows how to setup geometries, perform collision queries,
and how to do naive debug drawing.


<h2>Setting up the geometry</h2>

A polygonal mesh and a corresponding signed distance field are stored in the
local model frame of the object. Initially one have to setup the signed distance field geometry type by specifying the polygonal mesh type and the signed distance field map type, this could for instance be done by writing

<pre>
typedef OpenTissue::PolyMesh&lt;&gt;            mesh_type;
typedef OpenTissue::Map&lt;float&gt;            map_type;
typedef OpenTissue::SDF_Geometry&lt;mesh_type,map_type&gt;   sdf_geometry_type;
</pre>

Two functions are provided for initialization of a signed distance field
geometry type.

<pre>
sdf_init_geometry(...)
sdf_semiauto_init_geometry(...)
</pre>

As the naming convention indicates the second function offers semi automatic
initialization.

The difference is that the second method automatically computes a signed
distance field from the specified polygonal mesh. The function tries to be
clever about picking the best resolution. The user have the option to specify
the maximum allowed resolution, thus proving one with the means of controlling
how much memory is used.

The first method is ideal if one already have pre-computed the signed distance
field corresponding to the polygonal mesh (perhaps one have stored this on
disk).

A typical initialization of the signed distance geometry type would look like:

    #include <OpenTissue/utility/get_environment_variable.h>
    #include <OpenTissue/mesh/mesh.h>

    ...

    sdf_geometry_type geometry;
    string datapath = get_environment_variable( "DATATISSUE" );
    string meshfile = datapath + "/obj/pointy_tmp.obj";
    mesh_type mesh;
    mesh_obj_read( meshfile, mesh );
    double edge_resolution = 0.01;
    bool face_sampling = true;
    sdf_semiauto_init_geometry(mesh,edge_resolution,face_sampling,geometry);


<h2>Collision Queries</h2>


To perform a collision query between two signed distance field geometries one
would have to specify the world coordinate position of the two geometries, this
is done using a coordinate system type like:

<pre>
typedef OpenTissue::coordsys<real_type>   coordsys_type;

coordsys_type wcsA;
coordsys_type wcsB;

... set wcsA and wcsB to something ...
</pre>



Now the collision query is performed by writing

<pre>
sdf_geometry_type A;
... init A ..

sdf_geometry_type B;
... init B ..

real_type envelope = 0.01
collision_sdf_sdf(wcsA,wcsB,A,B,contacts,0.01);
</pre>

The envelope argument specifies a threshold distance, if objects are within this
distance then contact points will be generated. The contacts argument is a
contact point container upon return it will hold all the contact points that is
generated between objects A and B.

The contact point container could be declared as

<pre>
class contact_point_type
{
public:
vector3_type  m_n;        ///< The contact normal.
vector3_type  m_p;        ///< The contact point.
real_type     m_distance; ///< The penetration depth/separation distance measure.
};

typedef std::vector< contact_point_type > contact_point_container;

contact_point_container m_contacts;
</pre>

Any container type having a push_back method can be used, also any contact point
type can by used as long as it have the three members shown above.

Note by convention returned contact normals is specified in the world coordinate
system and the normals always points from object A towards object B. The contact
point is also specified in the world coordinate system, and the distance measure
is given along the contact normal.

<h2>Debug drawing</h2>

To render the sampling points of the signed distance field geometry, simply
invoke the debug drawing function
<pre>
glColorPicker(0.8, 0.1, 0.8);
sdf_debug_draw_sampling(geometry);
</pre>

Remember to setup any color before invoking the drawing function, also note that
the sampling is rendered in the local frame of the object. Thus to see sampling
points at correct position one could write

<pre>
glPushMatrix();
glTransform(wcsA);
glColorPicker(0.8, 0.1, 0.8);
sdf_debug_draw_sampling(A);
glPopMatrix();
</pre>

There is also basic functionality for drawing the bounding volume hierarchy
(BVH) data structure that is stored inside the signed distance field geometry
type. In order to do this one should call the following function

<pre>
unsigned int depth = 1;
sdf_debug_draw_bvh(geometry, depth);
</pre>

The depth argument specified the level at which the bounding volumes (BVs) of
the BVH. This function perform its drawing in the local frame of the object and
one needs to specify the wanted color of the drawn BV prior to invocation.
