==The Bounding Volume Hierarchy Data Structure==
A bounding volume hierarchy (BVH) is implemented as a tree structure,
where each node corresponds to a bounded volume. If a node contains
geometry as well then it is denoted as a annotated bounding volume
node. Usually only leaf nodes are annotated with geometry.

You gain access to the BVH data structure by including the header
file:
<pre>
#include<OpenTissue/collision/bvh/bvh.h>
</pre>

The idea behind a bounding volume hierarchy is to do a space
partitioning of an object. Thus a node corresponds to a partition of
the objects geometry. Children of a node corresponds to sub-partitions
of the geometry of the node.

To work probably for collision queries the bounded volume of a node,
must enclose the geometry represented by the children of that node.
Sometimes this requirement is eased and the bounded volume is required
to only enclose the bounded volumes of the children. Thus creating a
looser fitting bounded volume.

A bounding volume hierarchy of a point distribution using OBBs could
be created by writing:
<pre>
typedef OBB volume_type;
typedef vector3<double> geometry_type;
typedef BVH<volume_type,geometry_type> bvh_type;
bvh_type bvh;
</pre>

In general the geometry type could be anything. In some cases one
would want to store pointers to faces in a polygonal mesh. In which
case it would be more appropriate to write:
<pre>
typedef OBB volume_type;
typedef polymesh_type::face_type * geometry_type;
typedef BVH<volume_type,geometry_type> bvh_type;
bvh_type bvh;
</pre>

In this case the annotated bounding volume nodes holds a pointer to a
mesh face. This mean that you decide what volume type and geometry
type should be.  You can pretty much pick any type you want. Just
remember that a BVH makes it own copy of whatever geometry type you
give it. What does this mean? Well, consider the case of the point
distribution here the actual points are stored inside the annotated
bounding volume nodes. In case of the polygonal mesh if we have used
<pre>
typedef polymesh_type::face_type geometry_type;
</pre>
the annotated bounding volumes would store faces instead of
points.

Once the BVH have been created it is possible to iterate the nodes using a
bv_traversal_iterator, that is
<pre>
bvh_type::bv_traversal_iterator node = bvh.begin();
bvh_type::bv_traversal_iterator end  = bvh.end();
for(;node!=end;++node)
....
</pre>

The nodes will be processed in a breadth-first manner. A pointer to
the root bounding volume can be obtained by writing
<pre>
bvh_type::bv_ptr_type root = bvh.root();
</pre>

Observe that a bounding volume node pointer type have been defined in
the BVH type. This hides the fact that the BVH class manages it's
memory allocation using boost::shared_ptr. The bv_ptr_type can be used
in very much the same way as if one had written
<pre>
bvh_type::bv_type * root = .....
</pre>

If the total number of nodes is wanted the bounding volume hierarchy proves a
size-member for this
<pre>
unsigned int n = bvh.size();
</pre>

Likewise an empty-member is provided for testing whether a BVH contains any
nodes at all.
<pre>
bool empty = bvh.empty();
</pre>

When creating nodes in a bounding volume hierarchy it can happen in one of two
ways:
# Either a node is created as a child of a specified parent node.
# Or a node is created as the parent of a specified collection of nodes, which becomes the children nodes of the newly created node.

The first way is used when constructing a bounding volume hierarchy in a
top-down manner, whereas the later is used in bottom-up methods. The first way
is done using
<pre>
bv_ptr_type parent = ....
bool annotated = ....
bv_ptr_type bv =  bvh.insert(parent,annotated);
</pre>

The boolean flag indicates whether an annotated bounding volume node is created
or not. Observe that a bounding volume node is a base class of the annotated
bounding volume node.

Given a base pointer to any bounding volume node, one can test whether it is an
annotated bounding volume node or not by using the has_geometry()-method. That
is
<pre>
bv_ptr_type some_bv = ...
if(some_bv->has_geometry())
annotated_bv_ptr_type annotated = boost::static_pointer_cast<annotated_bv_type>(some_bv);

</pre>

Only annotated bounding volumes will return the value true when invoking the
has_geometry() method.

The code for creating a node in a bottom-up manner is as follows
<pre>
bv_ptr_container children = .....
bool annotated = ....
bv_ptr_type bv =  bvh.insert(children,annotated);
</pre>

If a bounding volume node has no children then it can be deleted by invoking
<pre>
bv_ptr_type target = ....
bool success = bvh.remove(target);
</pre>

The return value indicates whether the node was removed or not. To remove all
nodes at once just invoke the clear method on the bounding volume hierarchy
class.
<pre>
bvh.clear();
</pre>


===The Nodes===
A bounding volume node provides access to the bounding volume hierarchy
which it belongs to
<pre>
bv_ptr_type bv = ....
bvh_ptr_type owner = bv->owner();
</pre>

Also it provides access to root node and parent node
<pre>
bv_ptr_type root = bv->root()
bv_ptr_type parent = bv->parent()
</pre>

One can obtain the number of children nodes by invoking the method
<pre>
unsigned int n = bv->size();
</pre>
or the acronym ``bv->degree()''. This also often termed the cardinality or
branching factor in literature. A node stores a volume and one can gain access
to the volume using the method
<pre>
volume_type & volume = bv->volume();
</pre>

To make the code more readable the bounding volume node is supplemented with a
few query methods
<pre>
bool leaf = bv->is_leaf();
bool root = bv->is_root();
bool has_geometry = bv->has_geometry()
</pre>

These makes it easy to investigate local topology of any given node. Note that
the has_geometry() method can be used to determine whether a node is annotated or
not (as explained in the previous section).
Finally the bounding volume node provides iterators to iterate over all its children nodes.
<pre>
bv_iterator child = bv->child_begin();
bv_iterator end   = bv->child_end():
for(;child != end;++child)
....
</pre>

The annotated bounding volume node is inherited from the bounding volume node
and provides a few extra functionalities.
For instance on can insert the annotated geometry one-by-one using
<pre>
annotated_bv_ptr_type  bv = .....
geometry_type G = ....
bv->insert( G );
</pre>
Or a whole bunch of geometry at once using
<pre>
annotated_bv_ptr_type  bv = .....
geometry_container G = ....
bv->insert( G );
</pre>

Also one can iterate over the stored geometry using iterators
<pre>
geometry_iterator geometry = bv->geometry_begin();
geometry_iterator end      = bv->geometry_end();
for(;geometry!=end;++geometry)
....
</pre>

===Utilities===
All utilities can be accessed by including a single header file
<pre>
#include<OpenTissue/collision/bvh/util/bvh_util.h>
</pre>

Of course one can also include header-files of the individual utilities that are
used. This approach is taken in the following sections.

===Query Sets of Nodes===
The include header
<pre>
#include<OpenTissue/collision/bvh/util/bvh_get_all_nodes.h>
</pre>
contains a template function for extracting all nodes of a BVH into a container,
like this
<pre>
bvh_type::bv_ptr_container nodes;
bvh_get_all_nodes(m_bvh,nodes);
</pre>

In a similar fashion other queries can be performed, such as
<pre>
#include<OpenTissue/collision/bvh/util/bvh_get_leaf_nodes.h>
bvh_get_leaf_nodes(m_bvh,nodes);

#include<OpenTissue/collision/bvh/util/bvh_get_nodes_at_height.h>
unsigned int height = 2;
bvh_get_nodes_at_height(m_bvh,height,nodes);

#include<OpenTissue/collision/bvh/util/bvh_get_nodes_at_depth.h>
unsigned int depth = 2;
bvh_get_nodes_at_depth(m_bvh,depth,nodes);

#include<OpenTissue/collision/bvh/util/bvh_get_nodes_at_closest_height.h>

bvh_get_nodes_at_closest_height(m_bvh,height,nodes);
</pre>

===Constructing bounding volume hierarchies===
Currently there exist two ways for creating BVHs, either top-down or bottom-up.
In both cases you have to supply a constructor policy implementing specific
steps of the desired construction method. For a top-down construction you would
have to make a policy similar to this
<pre>
template<typename bvh_type>

class top_down_policy
{
protected:
....
public:

class partition_type
{
public:

bool annotated() const { ... }
unsigned int size() const { ... }
bool empty() { ... }
void split() { ... }
partition_iterator sub_partition_begin() { ... }
partition_iterator sub_partition_end() { ... }

void fit(bv_ptr bv) { ... }
};

public:

partition_type all() { ... }

template<typename iterator>
void init(iterator begin,iterator end){ ... }
};
</pre>
Then you can create your top down algorithm type like this
<pre>
#include <OpenTissue/collision/bvh/util/top_down_constructor/bvh_top_down_constructor.h>
typedef BVHTopDownConstructor< bvh_type, top_down_policy<bvh_type> >   constructor_algorithm;

</pre>
And finally you can build a bvh like this
<pre>
constructor_algorithm  constructor;
constructor.run(geometry.begin(),geometry.end,m_bvh);
</pre>

Bottom-up construction works in a similar fashion, except that the bottom-up
policy must define other types and methods.

===Refitting a bounding volume hierarchy===
Working with deformable objects, requires refitting of the BVH before a
collision query can be performed. Again this functionality is provided with the
help of policies.
<pre>
template<typename bvh_type_>
class refitter_policy
{
public:

void refit(bv_ptr bv)  { ...  }
};
</pre>
Now the policy can be put to use as follows
<pre>
#include <OpenTissue/collision/bvh/util/bvh_bottom_up_refitter.h>

typedef OpenTissue::BVHBottomUpRefitter< refitter_policy<bvh_type> >  refitter_algorithm;
refitter_algorithm refitter;
refitter.run(m_bvh);
</pre>

===Collision Queries with bounding volume hierarchies===

The mechanism of collision queries is no different than construction or
refitting. A policy must be defined for how the specific details of bounding
volume overlaps etc.. should be done.
<pre>
template<typename bvh_type_ >

class collision_policy
{
public:

template<typename result_container>
void reset(result_container & results)    { ...   }

bool overlap(coordsys_type const & A2B, bv_ptr bvA, bv_ptr bvB)    { ...  }
};
</pre>
And next
<pre>
#include <OpenTissue/collision/bvh/util/bvh_model_collision_query.h>
typedef BVHModelCollisionQuery< collision_policy<bvh_type> > collision_query_algorithm;
collision_query_algorithm   query;
query.run(A2B,bvhA,bvhB,results);
</pre>

===Outstanding Issues===
The bottom-up framework could be improved in three ways
# A better graph data structure (possible boost graph library).
# A better heap data structure (currently sorted lists are used).
# Allowing bottom-up policies to extend graph with dynamic/static properties needed to the given policy.

===Planned Future Extensions===
====The Packed BVH Library.====
The current BVH library is very generic and great for working with the BVH
topology in a dynamic way. However, once the BVH of an object have been built it
can be packed into a more efficient storage scheme using arrays. Thus, the
packed BVH library will contain methods for converting a ordinary (i.e.
unpacked) BVH into a packed BVH. However, it will not contain any methods, like
the top-down or bottom-up construction methods, for building a packed BVH.

The packed BVH will essentially be a fixed size array implementation, like this
<pre>
PackedBVH
+--------------------------------------------------------------+
|                                                              |
|  node_array:                                                 |
|  +--------------------------------------------------------+  |
|  |   |         |   |                                      |  |
|  |   |   ...   |   |                ........              |  |
|  |   |         |   |                                      |  |
|  +--------------------------------------------------------+  |
|                                                              |
|                                                              |
|  geometry_array:                                             |
|  +--------------------------------------------------------+  |
|  |   |         |   |                                      |  |
|  |   |   ...   |   |                ........              |  |
|  |   |         |   |                                      |  |
|  +--------------------------------------------------------+  |
|                                                              |
+--------------------------------------------------------------+
</pre>

The geometry array simply stores all the geometry. The node array stores the
bounding volume nodes, and these will have the following layout:
<pre>
node_type:

+-----------------------------------------+
|                 traits                  |  user specified
+-----------------------------------------+
|         cached volume_type (optional)   |  user specified
+-----------------------------------------+
|              volume_type                |  user specified
+-----------------------------------------+
|           query_time_stamp              |  long
+-----------------------------------------+
|           parent_node_idx               |  unsigned int
+-------------------+---------------------+
| left_node_idx     |  right_node_idx     |  unsigned int
+-------------------+---------------------+
| left_geometry_idx |  right_geometry_idx |  unsigned int
+-------------------+---------------------+
</pre>

Currently there is no plan to store BVH owner information, nor node idx
information.

The time stamp is intended to be used for caching optimizations in collision
queries. Thus, avoiding unnecessary model update transformations on already
transformed volume types.

The index members must always be set such that
<pre>
node_idx < left_node_idx <= right_node_idx
left_geometry_idx <= right_geometry_idx
</pre>
Here node_idx corresponds to the node_array entry of the node. The node indices
indicate the children nodes, and the geometry indices indicate the geometry
coverage of the node.

Note that the ranges given by these indices forms consecutive sub-arrays. The
storage layout corresponds to a pre-order lay-down of the data in a BVH.
<pre>
unpacked_bvh2packed_bvh(unpacked_bvh_type U, packed_bvh_type P)
{
pack_array(P.root,0,0,0)
}

pack_array(
bv_ptr_type bv, idx_type idx,idx_type & nxt_free_node,idx_type & nxt_free_geometry)
{
node_array[idx] = bv
if bv is leaf then
geometry_array[nxt_free_geometry] = bv.geometry
bv.left_geometry_idx  = nxt_free_geometry
bv.right_geometry_idx = nxt_free_geometry
nxt_free_geometry = nxt_free_geometry +1
end if

n = number of children of bv
bv.left_node_idx  = nxt_free_node + 1
bv.right_node_idx = nxt_free_node + n
next_free_node += n
for i=0, i < n
pack_array(bv.child[i],bv.left_node_idx+i,nxt_free_node,nxt_free_geometry)
next i
}
</pre>

Afterwards, the geometry coverage is propagated from leaves to the
root node. It is implicitly assumed that only leaves contains geometry
and each leaf contain exactly one geometry (If you need something else
use the unpacked BVH library). Keeping track of geometry coverage of
non-leaf nodes is an advantage for top-down refitting of a packed BVH.
Note that the node in the 0'th entry is always the root node.

The collision queries of a packed BVH is similar to the ones from the
unpacked BVH library. However they are different in two respects.
#  They do not use dynamic data structures (queues) for the traversal, they used fixed-size arrays which is re-allocated using a lazy memory allocation strategy, whenever the traversal depth exceed the current array capacity.
#  They have the optional inbuilt capability to support caching of model update transforms.

Note (2) can be done with the unpacked BVH library as well. But, an
end-user have to implement it by him self using the bounding volume
traits. The packed model collision query will look something like:
<pre>
void model_collision_query(A2B,bvhA,bvhB,results)
{
static long query_time_stamp = 0;
++query_time_stamp;

idx_type work[N];
idx_type cur = 0;
work[cur++] = 0
work[cur++] = 0
while(cur)
{
node_type A = bvhA.node_array[ work[ cur--] ]
node_type B = bvhB.node_array[ work[ cur--] ]

//--- Caching exploit
bool test = false;
if A.query_time_stamp = query_time_stamp
test = overlap(A.cached_volume,B.volume)
else
A.cached_volume = A.volume
test = overlap(A2B,A.cached_volume,B.volume);
A.query_time_stamp = query_time_stamp
end if

if(!test)
continue;
end if

if(leaf A and leaf B)
report(A2B,A,B;results)
continue
end if

//--- Lazy allocation
m = A.right_node_idx - A.left_node_idx + B.right_node_idx - B.left_node_idx +  2
if cur + m > N
reallocate work[N]

for (i = A.left_node_idx;i < A.right_node_idx)
work[cur++] = i
for (i = B.left_node_idx;i < B.right_node_idx)
work[cur++] = i
}
}
</pre>
Similar for other types of queries.

The packed BVH is designed to keep memory allocation at a minimum,
re-use computations if possible and exploit data locality. The
unpacked BVH allocates nodes on the heap, which means that siblings
can be scattered in memory. The packed BVH keeps siblings packed close
together in memory and tries to look them up in a consecutive sequence
only.

====Advanced Topics====
Implementing efficient contact determination of polygonal meshes. How
should one implement the report method in a collision query policy?
This is the question we will address in this section.
