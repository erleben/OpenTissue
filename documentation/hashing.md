= The Optimal Spatial Hashing Programming Guide =


== Introduction ==

Grid-based collision detection can be extremely fast and efficient in cases, where many objects are of the same size. OpenTissue implements a spatial hashing collision detection library for such cases. The library is used simply by including the header

<nowiki>#include<OpenTissue/collision/spatial_hashing/spatial_hashing.h></nowiki><br />

In the following we will outline the general usage of the spatial hashing library and defer details to later sections. In order to use the library a collision policy must be created and passed to a query type, e.g. as

template< ... ><br />  class collision_policy<br />  {<br />     ... a whole lot of things ...<br />  };<br /><br />  typedef collision_policy< ... > policy;<br />  typedef PointDataQuery<typename policy::hash_grid, policy> query_type1;<br />

The collision policy must define various types and implement various methods, which will be explained in more detail later. The first template argument defines the underlying container used for storing the grid. In the above example we used a so-called point query. There are two other query types

typedef LineDataQuery<typename policy::hash_grid, policy> query_type2;<br />  typedef AABBDataQuery<typename policy::hash_grid, policy> query_type3;<br />

The point-query is intended for testing point data against whatever data already mapped into the grid. Line queries test line-segments against the grid, and AABB queries test AABBs against the grid. To use a query, first the spatial grid must be initialized, which is typically done as follows

query_type1 point_query;<br />  point_query.init(data.begin(),data.end());<br />

The init method performs a statiscal analysis of the data, and tries to set grid spacing and number of hash cells to the best possible values. Alternatively, these values can be set manually as follows

unsigned int size = ...<br />  real_type spacing = ...<br />  point_query.resize( size );<br />  point_query.set_spacing( spacing  );<br />

To perform a collision query the function operator should be invoked as follows

typename policy::result_container   results;<br />  point_query( query.begin(), query.end(), data.begin(), data.end(), results, typename query_type1::all_tag() );<br />

This will test all the query data (i.e. points) against the specified data for collisions. The results will be reported in the supplied results-container and can be iterated upon return. The last argument is a tag telling, how the spatial query should handle hash-collisions. It can be costly to guard against hash-collisions, so if reports on multiple collisions are not needed, then the hash-collision guarding may be turned off by using the no_collision_tag() instead of the all_tag().

To repeat a query with altered or new query points, but against the same data, simply write

point_query( query.begin(), query.end(), results, typename query_type1::all_tag() );<br />

If one only has a single query data type, then one can write

...query_type q = ....;<br />  point_query( q, results, typename query_type1::all_tag() );<br />

Finally, the spatial query supports re-mapping of data by writing

point_query( data.begin(), data.end() );<br />

----

== The Hash Grid ==

The hash grid class stores an infinite, uniform 3D grid as a 1D hash tabel. It is tailored to work in 3D. However, an end user could use it in 2D or even 1D by ignoring second or third coordinates (i.e. set them all to the same value). The hash grid class is located in the header

#include<OpenTissue/collision/spatial_hashing/hash_grid.h><br />

The hash grid needs a hash function, which must supply the following interface

typedef ... size_type<br />  size_type operator(int ,int , int)const<br />  size_type size()const<br />  void resize(size_type)<br />

As of this writing, OpenTissue implements four different hash functions to be used together with the hash grid. These are located in the header files

#include<OpenTissue/collision/spatial_hashing/hash_functions/grid_function.h><br />  #include<OpenTissue/collision/spatial_hashing/hash_functions/prime_number_function.h><br />  #include<OpenTissue/collision/spatial_hashing/hash_functions/random_array_function.h><br />  #include<OpenTissue/collision/spatial_hashing/hash_functions/shifted_golden_mean_function.h><br />

It is usually a good idea to try different hash functions in order to find the one that yields the best performance. Here is an example of typical usage

typedef vector3<real_type>  point_type;<br />   typedef ...  data_type;<br />   typedef GridHashFunction  hash_function1;<br />   typedef HashGrid< point_type, vector3<int>, data_type, hash_function1>  hash_grid;<br />

The other hash functions could be typedefs in a similar fashion

typedef RandomArrayHashFunction       hash_function2;<br />  typedef PrimeNumberHashFunction       hash_function3;<br />  typedef ShiftedGoldenMeanHashFunction hash_function4;<br />

Important: The hash function is responsible for picking the proper hash table size. Thus, if resize is invoked and the implemented hash function for instance implements something like:

m_size  = new_size + (new_size % 10)<br />

then the hash grid will automatically make sure that m_size is used to allocate the number of hash cells. The size() method is expected to return the value of m_size.

The operator(int,int,int) should convert a discretized point into a 1D hash key. The hash key value is expected to be within the interval 0..m_size-1.

Observe that a grid cell is not the same as a hash-cell. Data inside a grid cell is mapped to a hash-cell, and this is not a one-to-one mapping. In fact, multiple grid cells can be mapped to the same hash-cell (this is called a hash-collision). To reduce the chance of this, it is often a good idea to increase the hash-table size, which will decrease the chance of hash-collisions. For point type data, the grid spacing should usually be set to the average size of the query data used. For volumetric data, the grid spacing should usually be set to the average size of the volumetric data.

----

== The Spatial Query ==

The spatial query class is a generic type for all types of queries on a spatial hash grid. All queries should be inherited from this class. The spatial query class is located in the header

#include<OpenTissue/collision/spatial_hashing/spatial_query.h><br />

Currently OpenTissue implements three different query types, these are all inherited from the spatial query class and are located in the header files

#include<OpenTissue/collision/spatial_hashing/hash_queries/point_data_query.h><br />  #include<OpenTissue/collision/spatial_hashing/hash_queries/line_data_query.h><br />  #include<OpenTissue/collision/spatial_hashing/hash_queries/aabb_data_query.h><br />

Their major difference lie in how the hash-grid cells are traversed, when testing a query type for collision. We refer the interested reader to the above header files for details.

The report behavior are controlled by passing along a tag to the queries. The tag indicates, whether the query will guard against reporting multiple but identical (data, query) pairs. Optionally, the end-user can implement such guarding himself in the collision_policy::report method. The two tags are:

* The report all tag: all_tag, when this tag is used as report type, then all grid overlaps are reported, even when redundant. As an example, imagine a data_type is mapped into two different grid cells. If a query type overlaps both grid cells, then collision_policy::report is invoked twice with the same data and query arguments.
* The no collision report tag: no_collisions_tag. In contrast to all_tag, when this tag is used as report type, then the query will guard against collisions. That is, even though the same data and query are found more than once during a query, collision_policy::report is only invoked once.

An end-user must supply a collision_policy type, which defines two methods:

reset(results)<br />  report(data_type,query_type, results_container)<br />

and depeding on the type of query, the collision policy also supplies all or a subset of following methods

point_type position( data_type )       // only for point_data_query<br />  point_type min_coord data_type )       // only for aabb_data_query<br />  point_type max_coord( data_type )      // only for aabb_data_query<br />  point_type min_coord( query_type )     // for points, line and aabb data queries<br />  point_type max_coord( query_type )     // for points, line and aabb data queries<br />  point_type origin( data_type )         // only for line_data_query<br />  point_type destination( data_type )    // only for line_data_query<br />

The report method must implement a collision test, such as e.g. a point in box test. The report method is responsible for adding collision test results to the result container, and the report method is also responsible for guarding against self-collisions and possible double reported pairs (with order exchanged).

The reset method is responsible for making the results container ready for a new query, such as e.g. to allow for the re-use of allocated memory for collision results.

The spatial query expects a hash_grid type as a template argument. This hash_grid type must support the following interface:

typedef ... cell_type<br />  typedef ... triplet_type<br />  typedef ... point_type<br />  typedef ... real_type<br />  typedef ... data_type<br /><br />  resize(new_size)<br />  set_spacing(new_spacing)<br />  cell_iterator  begin()<br />  cell_iterator  end()<br />  triplet_point get_triplet(point_type)<br />  cell_type & get_cell(triplet_type)<br />

Finally, hash cells must support the following interface:

m_query_stamp<br />  data_iterator begin()<br />  data_iterator end()<br />  empty()<br />  add( data_type )<br />

----

== A Collision Policy Example ==

Let us implement a collision policy. We want to find the tetrahedra enclosing a surface mesh vertex. We start by declaring a collision policy class and defining some convenient types

class collision_policy<br />    {<br />    public:<br /><br />      typedef typename surface_mesh::vertex_type        vertex_type;<br />      typedef typename volume_mesh::tetrahedron_type    tetrahedron_type;<br />      typedef double                                    real_type;<br />      typedef vector3<real_type>                        point_type;<br />      typedef vertex_type*                              data_type;<br />      typedef tetrahedron_type                          query_type;<br />

Next we will define the hash function and hash grid types that we want to use in our spatial query. These do not have to be declared inside the collision policy, but doing so makes code maintenance easier.

typedef OpenTissue::GridHashFunction                        hash_function;<br />      typedef OpenTissue::HashGrid< point_type, OpenTissue::vector3<int>, data_type, hash_function> hash_grid;<br />

Hereafter it is convenient to define the result type of a collision query. We will define this by an inner class. For this example we decided to report pointers to the vertex (query type) and the tetrahedron (data type) enclosing the vertex together with the barycentric coordinates of the vertex wrt. the tetrahedron. This looks like

class result_type<br />      {<br />      public:<br />        vertex_type * m_data;<br />        tetrahedron_type * m_query;<br />        real_type  m_w0;<br />        real_type  m_w1;<br />        real_type  m_w2;<br />        real_type  m_w3;<br />      };<br /><br />      typedef std::list<result_type>  result_container;<br />

For this example we need to support the interface needed by a point query type, which means that we must be able to extract the position of the data type (a surface mesh vertex pointer) and the AABB corners of the query type (a volume mesh tetrahedron).

public:<br /><br />      point_type position(data_type const & data) const <br />      {<br />        return data->m_coord;<br />      }<br /><br />      point_type min_coord(query_type const & query) const <br />      {<br />        using std::min;<br /><br />        point_type & p0 = query.i()->m_coord;<br />        point_type & p1 = query.j()->m_coord;<br />        point_type & p2 = query.k()->m_coord;<br />        point_type & p3 = query.m()->m_coord;<br />        return min( p0, min( p1 , min( p2, p3) ) );<br />      }<br /><br />      point_type max_coord(query_type const & query) const <br />      {<br />        using std::max;<br /><br />        point_type & p0 = query.i()->m_coord;<br />        point_type & p1 = query.j()->m_coord;<br />        point_type & p2 = query.k()->m_coord;<br />        point_type & p3 = query.m()->m_coord;<br />        return max( p0, max( p1 , max( p2, p3) ) );<br />      }<br />

Finally we must implement the reset and report methods.

void reset(result_container & results)  {    results.clear();  };<br />      void report(data_type const & data, query_type const & query,result_container & results)<br />      {<br />        point_type & pi = query.i()->m_coord;<br />        point_type & pj = query.j()->m_coord;<br />        point_type & pk = query.k()->m_coord;<br />        point_type & pm = query.m()->m_coord;<br />        point_type & p  = data->m_coord;<br /><br />        real_type delta = 10e-5;<br />        real_type lower = - delta;<br />        real_type upper = 1.+ delta;<br />        result_type result;<br />        barycentric(pi,pj,pk,pm,p,result.m_w0,result.m_w1,result.m_w2,result.m_w3);<br />        if(<br />          (result.m_w0>lower)&&(result.m_w0<upper)<br />          &&<br />          (result.m_w1>lower)&&(result.m_w1<upper)<br />          &&<br />          (result.m_w2>lower)&&(result.m_w2<upper)<br />          &&<br />          (result.m_w3>lower)&&(result.m_w3<upper)<br />          )<br />        {<br />          data->m_tag = 1;<br />          result.m_data = const_cast<vertex_type*>( data );<br />          result.m_query = const_cast<tetrahedron_type*>( &query );<br />          results.push_back( result );<br />          return;<br />        }<br />      }<br />    };<br />

That is all we need, now we can use our collision policy to setup a point query

typedef PointDataQuery<typename collision_policy::hash_grid, collision_policy> query_type;<br />

----

== Examples ==

Optimal spatial hashing is used many places in OpenTissue. Here is a short list of header files, where example usage may be found

<nowiki>#include<OpenTissue/t4mesh/util/t4mesh_mesh_coupling.h></nowiki><br /><nowiki>#include<OpenTissue/dynamics/cfd/sph/sph_system.h></nowiki><br /><nowiki>#include<OpenTissue/dynamics/multibody/CD/retro_spatial_hashing.h></nowiki><br /><nowiki>#include<OpenTissue/t4mesh/util/thin_shell/policies/bisection_adaptive_extrusion.h></nowiki>
