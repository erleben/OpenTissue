# The Tetrahedra Mesh Programming Guide

The tetrahedra mesh data structure is designed specially for two purposes: Firstly, it should maintain a valid topology of the mesh at all times, such that the connectivity of nodes and tetrahedra always are valid. Secondly, the tetrahedra mesh data structure ensures that the global indexing of nodes (0..N-1) and tetrahedra (0..T-1) always are a compact range starting from zero to the maximum number minus one.

Obviously removing entities (nodes or tetrahedra) alters the global index ranges, thus end users cannot trust previously stored indices of entities in their own applications.


## The Tetrahedra Mesh Data Structure

The Tetrahedra mesh data structure is defined in the include header file

    #include<OpenTissue/t4mesh/t4mesh.h>

The data structure is a template class, which takes several arguments. A typical declaration looks like

    T4Mesh<> mesh;

This will create a default tetrahedra mesh, where every node has a single 3D coordinate vector named, m_coord. This default behavior is obtained from two default trait classes defined in the header file

    #include<OpenTissue/t4mesh/t4mesh_default_traits.h>

There is no need to include this header file explicitly. The two classes in this header file defines all default data members of nodes and tetrahedra, which are the basic building blocks of a tetrahedra mesh.

New nodes can be inserted by invoking the insert method, e.g. as

    typedef t4mesh< > mesh_type;
    typedef mesh_type::node_iterator node_iterator;

    node_iterator i = mesh.insert();
    node_iterator j = mesh.insert();
    node_iterator k = mesh.insert();


The node iterator can be used to access data members, for instance when using default node traits, the coordinates of nodes may be changes by

    i->m_coord = vector3_type(1, 2, 3);

or one can simply supply the coordinates at insertion time

    node_iterator m = mesh.insert( vector3_type(3, 4, 5) );

There are two ways to create tetrahedra elements. Either by using iterators as follows,

    typedef mesh_type::tetrahedron_iterator   tetrahedron_iterator;
    tetrahedron_iterator t = mesh.insert( i, j, k ,m );

or if the indices of the nodes making up a tetrahedron are known, then the index values can be used directly

  tetrahedron_iterator t = mesh.insert( 0, 1, 2, 3 );

In a similar fashion one can lookup iterators to tetrahedron

  tetrahedron_iterator t = mesh.find( i, j, k, m);

Removing tetrahedra is done by

  tetrahedron_iterator t = ...
  mesh.erase( t );

Currently there is no support for removing nodes. However there is support for looking up iterators for nodes and tetrahedra either by

  unsigned int idx = ...
  node_iterator n = mesh.node( idx )

or by

  unsigned int idx = ...
  tetrahedron_iterator t = mesh.tetrahedron( idx )

The number of nodes and tetrahedra can be found by

  unsigned int nodes = mesh.size_nodes();
  unsigned int tetrahedra = mesh.size_tetrahedra();

Finally the tetrahedra mesh has support for iterating over nodes and tetrahedra as e.g.

  for(node_iterator n = mesh.node_begin();n!=mesh.node_end();++n)
    ...
    for(tetrahedron_iterator t = mesh.tetrahedron_begin();t!=mesh.tetrahedron_end();++t)
      ...

## Using Traits

In many cases one wants to extend or change the default traits for the t4mesh data structure. The default traits are defined in the header

    #include<OpenTissue/t4mesh/t4mesh_default_traits.h>

This header file is automatically included in the header file

    #include<OpenTissue/t4mesh/t4mesh.h>

As an example, say we want to create some sort of particle based simulation method, then we start by extending the node traits to contain relevant particle information.

class my_node_traits<br />  {<br />  public:<br />    typedef typename double              real_type;<br />    typedef typename vector3<real_type>  vector3_type;<br /><br />  public:<br /><br />    bool         m_fixed;       ///< Node is fixed.<br />    vector3_type m_prev_coord;  ///< old position.<br />    vector3_type m_coord;       ///< position.<br />    vector3_type m_v;           ///< velocity.<br />    real_type    m_mass;        ///< Total mass.<br />    vector3_type m_f;           ///< Accumulator of forces.<br />  };<br />

Note that we chose to use the name m_coord for the position. This makes it very easy to use the t4mesh I/O functions (more on this later) without creating a generic point container (also more on this later). For this example ,we really do not need any tetrahedron traits, thus

class my_tetrahedron_traits<br />  {<br />  public:<br />  };<br />

Now we can define our mesh type as

typedef   OpenTissue::t4mesh::T4Mesh< math_types, my_node_traits, my_tetrahedron_traits >  my_mesh_type;<br />

For this example we could better exploit inheritance to add a simulation method to our tetrahedra mesh. Here we add a Verlet integration time-step method

class MyVolumeMesh : public OpenTissue::t4mesh::T4Mesh< math_types, my_node_traits, my_tetrahedron_traits ><br />  {<br />  public:<br /><br />    typedef OpenTissue::t4mesh::T4Mesh< math_types, my_node_traits, my_tetrahedron_traits >  base_class;<br />    typedef typename my_node_traits::real_type                  real_type;<br />    typedef typename my_node_traits::vector3_type               vector3_type;<br />    typedef typename base_class::node_iterator                  node_iterator;<br /><br />  public:<br /><br />    void step(real_type const & dt)<br />    {<br />      real_type dt2 = dt*dt;<br />      real_type inv_2dt = 1.0/ (2.0*dt);<br />      node_iterator node  = this->node_begin();<br />      node_iterator end   = this->node_end();<br />      for(node=begin;node!=end;++node)<br />      {<br />        if(node->m_fixed)<br />          continue;<br />        vector3_type tmp = node->m_coord;<br />        node->m_coord = 2.0*node->m_coord - node->m_prev_coord + (dt2/node->m_mass) * ( node->m_f );<br />        node->m_v = (node->m_coord - node->m_prev_coord)* inv_2dt;<br />        node->m_prev_coord = tmp;<br />      }<br />    }<br />  };<br />

To use this mesh one would have to compute the nodal forces before invoking the time-stepping method.


## Default Point Container

The default point container class is a utility class which can be used to make the coordinates of the nodes in a tetrahedra mesh appear as a point container, i.e. as though the coordinates are stored as

std::vector<vector3_type> coordinates;<br />

and accesses as

coordinates[node->idx()]<br />

instead of

node->m_coord<br />

Many algorithms in OpenTissue have been implemented in such a way that they do not rely on nodes to have a m_coord member. Instead coordinates are passed as point containers. This utility make it convenient to use these algorithms on nodes, where coordinates are stored in m_coord member.

The default point container class is located in the header file

    #include<OpenTissue/t4mesh/t4mesh_default_traits.h>

and it is included automatically by including the header

    #include<OpenTissue/t4mesh/t4mesh.h>

The default point container class is used as follows

default_point_container<t4mesh_type> points(&mesh);<br />  points[idx] = vector3_type(1, 2, 3);<br />


## T4Mesh Face Extraction

Faces are not stored explicitly in the t4mesh, but sometimes it is useful to extract the boundary faces of a t4mesh. OpenTissue provides a utility class for exactly this purpose. The class is located in the header

    #include<OpenTissue/t4mesh/t4mesh_t4boundary_faces.h>

Let us work through a small example to learn how to use the utility. First we need a volume mesh

typedef OpenTissue::t4mesh::T4Mesh<....> mesh_type;<br />  mesh_type my_mesh;<br />  ... do something with my_mesh ...<br />

Then we can setup the face boundary type, and finally we extract the boundary faces by<br />

typedef OpenTissue::t4mesh::T4BoundaryFaces< mesh_type > boundary_type;<br />  boundary_type boundary(my_mesh);<br />

Observe that the extraction is done using the constructor of the face boundary type. This means that if one alters the volume mesh topology, then the face boundary may come out of sync with the volume mesh. In such cases the face boundaries must be re-extracted. After having extracted the boundary faces they may be iterate over as follows

typedef  boundary::face_iterator face_iterator;<br />  for(face_iterator face=boundary.begin();face!=boundary.end();++face)<br />  {<br />    node_type & ni = my_mesh.node( face->idx0() );<br />    node_type & nj = my_mesh.node( face->idx1() );<br />    node_type & nk = my_mesh.node( face->idx2() );<br />    ... do something with nodes ...<br />  }<br />

The boundary face types may be extended by using traits traits, which is done by defining a trait class

class my_face_traits : public OpenTissue::t4mesh::DefaultFaceTraits<br />  {<br />    public:<br />       ...<br />       color_type  m_color;<br />       ...<br />  };<br />

Now we just have to change the face boundary type

typedef OpenTissue::t4mesh::T4BoundaryFaces< mesh_type, my_face_traits > boundary_type;<br />

During iteration we can now access the new traits

typedef  boundary::face_iterator face_iterator;<br />  for(face_iterator face=boundary.begin();face!=boundary.end();++face)<br />  {<br />    face->m_color = ...<br />  }<br />


## T4Mesh Edge Extraction

Edges are not represented explicitly in a t4mesh, only nodes and tetrahedra are represented. OpenTissue provides a utility class for extracting all unique edges from a t4mesh by traversing the t4mesh and generating explicit edges.

    #include<OpenTissue/t4mesh/t4mesh_t4edges.h>

The workings of this utility class is exactly the same as the boundary face extraction tool (explained previously), we therefore just show a short example here

typedef OpenTissue::t4mesh::T4Edges < mesh_type > edges_type;<br />  edges_type edges(my_mesh);<br />  typedef edges_type::edge_iterator edge_iterator;<br />  for(typename edges_type::edge_iterator edge=edges.begin();edge!=edges.end();++edge)<br />  {<br />    node_type & ni = my_mesh.node( edge->idxA() );<br />    node_type & nj = my_mesh.node( edge->idxB() );<br />    ... do something ...<br />  }<br />


## T4Mesh IO functions

OpenTissue has its own XML-fileformat for storing t4meshes. The header files:

    #include<OpenTissue/t4mesh/io/t4mesh_xml_read.h #include<OpenTissue/t4mesh/io/t4mesh_xml_write.h>

defines read and write functions. These functions are used as follows

std::string input_filename = ...<br />  t4mesh_xml_read( input_filename, my_mesh);<br />  ... do something with my_mesh ...<br />  std::string output_filename = ...<br />  t4mesh_xml_write( output_filename, my_mesh);<br />

These versions of the I/O-functions assumes that the node traits has a m_coord data member. If not, you must supply a third argument, which is a point container. As an example say you decided to call your positional data member m_x0, that is

class my_node_traits<br />  {<br />  public:<br />    ...<br />    vector3_type m_x0;<br />    ...<br />  };<br />

Now a point container interface can be defined as follows

struct my_point_container<br />{<br />  my_point_container(mesh_type * mesh) : m_mesh(mesh) {};<br /><br />  vector3_type & operator[] (unsigned int const & idx)  {    return m_mesh->node(idx)->m_coord;  };<br /><br />  vector3_type const & operator[] (unsigned int const & idx)const  {    return m_mesh->node(idx)->m_coord;  };<br /><br />  void clear(){};<br />  unsigned int size()const{return m_mesh->size_nodes();};<br />  void resize(unsigned int){};<br /><br />  mesh_type * m_mesh;<br />};<br />

and the point container may be used as follows

my_point_container points(&my_mesh);<br />  OpenTissue::t4mesh::xml_read( input_filename,  my_mesh, points);<br />  OpenTisssue::t4mesh::xml_write( output_filename, my_mesh, points);<br />

You may even decide not to store coordinates inside nodes, but instead have them stored in an external data structure. This is done as follows,

std::vector<vector3_type> points(my_mesh.node_size());<br />  OpenTissue::t4mesh::xml_read( input_filename,  my_mesh, points);<br />  OpenTissue::t4mesh::xml_write( output_filename, my_mesh, points);<br />

OpenTissue also support importing TetGen files, the read function is defined in the header

    #include<OpenTissue/t4mesh/io/t4mesh_tetgen_read.h>

And it is used in completely the same fashion as the XML read function

OpenTissue::t4mesh::tetgen_read( input_filename,  my_mesh );<br />  OpenTissue::t4mesh::tetgen_read( input_filename, my_mesh, points);<br />


## T4Mesh OpenGL Drawing

OpenTissue provides a simple OpenGL visualization function of tetrahedra meshes. The function is located in the header

    #include<OpenTissue/utility/GL/gl_draw_t4mesh.h>

The function assumes that nodes have a m_coord data member. The function is invoked as follows

    OpenTissue::gl::DrawT4Mesh(m_mesh);

The function has two more arguments providing some more control over the visualization. The scale parameter defines an individual scaling of each tetrahedron, before it is drawn. The drawing mode parameter is usually either GL_POLYGON or GL_LINE_LOOP. The last is used for wireframe rendering.

    OpenTissue::gl::DrawT4Mesh(m_mesh,0.8,GL_LINE_LOOP);


## Mesh Coupling

The main idea behind mesh coupling is to create a multi-resolution mesh for the animation. The idea is to separate the visual geometry from the geometry used to compute the dynamics, such that highly detailed representation is used for visualization, while using a a coarse representation is used to compute the dynamics in order to reduce simulation time.

A technique for doing this is called mesh coupling or cartoon meshing. Below we describe, how it is used together with tetrahedral meshes. However, it is a general approach and can be used with other types of geometries, such as FFD lattices.

When using mesh coupling, the first step is to bind the vertices of the surface mesh to the tetrahedral elements of the volume mesh. Here we use a spatial hashing algorithm to find vertex tetrahedron pairs, where the vertex is embedded inside the tetrahedron. The actual test is done by first computing the barycentric coordinates of the vertex with respect to a tetrahedron in question. If each barycentric coordinate is greater than or equal to zero and less than equal to one, then the vertex is embedded in the tetrahedron. In practice we need to apply threshold testing to counter numerical imprecision. It may therefore happen that vertices lying close to a face of a tetrahedron gets reported twice: once for the tetrahedron embedding it, and once for the neighboring tetrahedron. The same happens if the vertex lies exactly on a face. Therefore, we first do a quick rejection test. If the vertex is already reported then it is simply ignored!

Before rendering each frame, we must update the vertex positions to reflect the underlying deformation of the tetrahedral mesh. This is done using the barycentric coordinates, such that the new vertex position is given by

<center> c = w0 p0 + w1 p1 + w2 p2 + w3 p3 </center>

Where p0, p1 ,p2, and p3 are the nodal coordinates of the tetrahedron which the vertex was bounded to.

If stiffness warping is used, then the element rotation, Re, can be used to update the undeformed vertex normal, n0, into the deformed vertex normal, n, by,

<center> n = Re n0 </center>

Often a tetrahedra mesh is used with a conservative coverage of the surface mesh. This guarantees that all vertices of the surface mesh are embedded inside one unique tetrahedron. However, mesh coupling can be used in cases, where one only have a partial coverage. The solution is to bind a vertex to the closest tetrahedron. Eventhough the vertex lies outside the tetrahedra mesh, the barycentric coordinates extend the deformation of the tetrahedra mesh beyond its surface.

The mesh coupling functions are located in the header file

    #include<OpenTissue/t4mesh/util/t4mesh_mesh_coupling.h>

In order to use it, one must bind a surface mesh (PolyMesh or TriMesh) to the volume mesh (t4mesh).

OpenTissue::t4mesh::T4Mesh<>                   volume_mesh;<br />  OpenTissue::polymesh::PolyMesh<>                surface_mesh;<br />  std::vector barycentric;<br />  std::vector bind_info;<br /><br />  ... create geometry of surface mesh ...<br />  ... create geometry of volume mesh  ...<br /><br />   OpenTissue::t4mesh::mesh_coupling::bind_surface(surface_mesh,volume_mesh,barycentric,bind_info);<br />

Now the surface mesh may be updated, when the volume mesh have changed its shape using,

OpenTissue::t4mesh::mesh_coupling::update_surface(surface_mesh,volume_mesh,barycentric,bind_info);<br />

This should be done prior to rendering the surface mesh.


## Delaunay Tesselator

This utility function provides a wrapper interface for QHull, which performs a delaynay triangulation a given set of points. The utility function is located in the header file

    #include<OpenTissue/t4mesh/util/t4mesh_delaunay_tesselator.h>

And it is invoked by supplying a container of points, that is

std::vector<vector3_type> points;<br />  ... create some points ...<br />  OpenTissue::t4mesh::delaunay_tesselator( points, my_mesh );<br />

One can use any container type that is desired.
