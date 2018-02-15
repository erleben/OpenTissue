## The Mesh Programming Guide
### Concepts and Terminology
The PolyMesh class provides methods for creating and removing vertices, edges,
and faces. These primitive elements are termed the mesh features.

A poly mesh is based on a kernel. A kernel is the actual storage of the mesh
features. The kernel is essentially a multi-index set. That is to say a kernel
provides an interface for accessing mesh features based on indices, handles,
and iterators.


The connectivity of the polymesh data structure is based on indices. Each mesh
feature set has its own unique index set:
<pre>
vertices = {v0 ,v1, v2, v3, ...}
halfedges = {h0 ,h1, h2, h3, ...}
edges  = {e0,e1,e2,....}
faces = {f0,f1,f2,....}
</pre>

Notice that the index 0 is present in each feature set, so is 1, 2 etc..
Indices are thus only unique within the feature set.

A handle is an unique key that can be used to lookup the corresponding mesh
feature in constant time. In terms of array based storage a handle would simply
be implemented as the number of the array entry holding the mesh feature. Thus
a handle provides random access to mesh features in constant time, but they can
not be used to iterate the mesh features. Iterators is a well known concept
these can be used to iterate over all vertices, edges, halfedges or faces.

The kernel type is responsible for defining and exposing index types, handle
types and iterator types. Besides the kernel type provides lookup methods,
handle validation tests etc..

Notice the semantics, a handle can be null this is a unique value meaning
that the handle explicitly points to nothing. A non-null handle may be both
valid and non-valid. If the non null handle identifies existing data in the
mesh, then the handle is defined to be valid. If no corresponding data exist
then the handle is non-valid.

Handles can be copied and remain valid, also handles remain valid upon
deletion of other mesh features in the mesh. A handle should only become
invalid if and only if the corresponding mesh feature of the handle is deleted.
This is unlike iterators and circulators which may become invalid upon deletion
and insertion.

A circulator is similar in concept to an iterator. It provides a means to
iterate over the 1-ring neighborhood of a vertex or a face. The major
difference between a circulator and an iterator is that a circulator does not
have a designated starting and end position. One simply iterates from one
neighboring mesh feature until the same feature is encountered again.

Finally, features support traits, this basically mean that the end user can
extend or change any types or data members on mesh features as he or she
pleases. The polymesh data structure only cares about topology and connectivity
of mesh features. Although it does have a vertex creation method, which takes a
coordinate vector argument. This was added for convenience and implies that all
algorithms in OpenTissue assumes that the position of a vertex can be accessed
through the vector data member m_coord. Other than this, there are no other
bindings at this time of writing.

### Code Examples
Now some code examples to get you up and running quickly. First we define a
mesh type, using default kernel and traits, and then we instantiate a mesh:

<pre>
typedef OpenTissue::polymesh::PolyMesh<> mesh_type;
mesh_type   m_mesh;
</pre>

Now add three new vertices:

<pre>
typedef typename mesh_type::math_types     math_types;
typedef typename math_types::vector3_type  vector3_type;

mesh_type::vertex_handle v0 = m_mesh.add_vertex(vector3_type(0,0,0));
mesh_type::vertex_handle v1 = m_mesh.add_vertex(vector3_type(-10,0,0));
mesh_type::vertex_handle v2 = m_mesh.add_vertex(vector3_type(-5,-10,0));
</pre>

Create a new face:

<pre>
typedef typename mesh_type::vertex_handle     vertex_handle;
typedef typename mesh_type::face_handle       face_handle;

vector<vertex_handle> vhandles(3);
vhandles[0] = 5;
vhandles[1] = 6;
vhandles[2] = 0;
face_handle f0 = m_mesh.add_face(vhandles.begin(),vhandles.end());
</pre>

Make a copy and an assignment:

<pre>
mesh_type tmp(m_mesh);
m_mesh.clear();
m_mesh = tmp;
</pre>

Remove the face

<pre>
m_mesh.remove_face(f0);
</pre>

Remove the vertices

<pre>
m_mesh.remove_vertex(v0);
m_mesh.remove_vertex(v2);
m_mesh.remove_vertex(v3);
</pre>

Or simply to remove everything in the mesh use:

<pre>
m_mesh.clear();
</pre>

To use a vertex circulator (or any other type of circulator):

<pre>
typedef typename mesh_type::vertex_iterator             vertex_iterator;
typedef typename mesh_type::vertex_halfedge_circulator  vertex_halfedge_circulator;

vertex_iterator v = m_mesh.get_vertex_iterator(v0);
vertex_halfedge_circulator circulator(* v),end;
for(;circulator!=end,++circulator)
{
...
}
</pre>

Or to iterate over a feature set write something like:

<pre>
typedef typename mesh_type::halfedge_iterator  halfedge_iterator;

halfedge_iterator begin = m_mesh.halfedge_begin();
halfedge_iterator end = m_mesh.halfedge_end();
halfedge_iterator halfedge = begin;
for(;halfedge!=end;++halfedge)
{
  ....
}
</pre>

To see if a handle is valid write

<pre>
vertex_handle v0 = .....
bool valid = m_mesh.is_valid_vertex_handle(v0);
</pre>

Null handles can be tested in two ways

<pre>
bool null = (v0 == m_mesh.null_vertex_handle);
</pre>

or

<pre>
bool null = v0.is_null()
</pre>

### Unsupported Features
* <p>Currently end users can not create mesh features with specific indices. This may change in the future, in order to better support progressive meshes. However it currently implies that indices keep on increasing. </p> <p>As an example if three vertices are entered into a newly instantiated mesh. Then they will have indices 0, 1, 2. Say the end user decides to delete one of these vertex, for instance 2, and then afterwards adds a new vertex, then the new indices would be 0, 1, 3. Similar if end user decided to delete vertex 1, then one would end up with 0, 2, 3. </p>
* There is currently no support for dynamic properties. However, and end user can make his or her own extensions by either implementing a new kernel type or exploiting the traits.
* End user is not allowed to create new edges directly (but direct deletion is allowed!). The main idea is that only vertices and faces are created and removed. If edge manipulation is needed then a utility class should be implemented for doing this (see the notes on back-door gateway design pattern, http://www.opentissue.org/wikitissue/index.php/Design_Patterns#Back-door_Gateway_Design_Pattern).

### Implementation Details
* This design uses a back-door gateway (http://www.opentissue.org/wikitissue/index.php/Design_Patterns#Back-door_Gateway_Design_Pattern) to access connectivity data on mesh features. This means that end users, which extend the polymesh library with new utilities should use the class polymesh_core_access to change and edit mesh features.
* Entire implementation is done in header files. Thus there is no need for end users to link with the entire OpenTissue (or even try to build OpenTissue) in order to use the mesh data structure.
* End users can use his or her own matrix-vector library simply by changing the math_types template argument for the mesh class.

### Notes
* The PolyMesh data structure was designed to strongly enforce two manifolds (with open boundaries). The PolyMesh can not represent degenerate data, such as two cones meeting in one vertex or three faces sharing a single edge. If you have such mesh data then you should use another type of mesh. For instance a triangle mesh.
