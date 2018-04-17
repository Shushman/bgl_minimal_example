#include <exception>
#include <memory>
#include <sstream>
#include <iostream>
#include <fstream>
#include <random>
#include <type_traits>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Dense>

namespace po = boost::program_options;


// Properties associated with vertex
struct VProps
{
  // Underlying state of vertex
  std::shared_ptr<Eigen::VectorXd> v_state;
};

// Consts to represent collision status of edge
const int FREE{1};
const int BLOCKED{-1};
const int UNKOWN{0};

struct EProps
{
  // Weight of edge
  double e_weight;

  // Collision status of edge
  int e_status;
};

// Graph type definitions
typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, VProps, EProps> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;

// Boost property maps needed to read in roadmap
// Property type specified as second template parameter
typedef boost::property_map<Graph, std::shared_ptr<Eigen::VectorXd> VProps::*>::type VPropStateMap; // For vertex state
typedef boost::property_map<Graph, double EProps::*>::type EPWeightMap;
typedef boost::property_map<Graph, boost::vertex_index_t>::type VertexIndexMap;


// The map used to decode the .graphml file and populate vertex states.
// Avoiding templates - working directly with VPropStateMap type
class RoadmapFromFilePutStateMap
{
public:
  typedef boost::writable_property_map_tag category;
  typedef typename boost::property_traits<VPropStateMap>::key_type key_type;
  typedef std::string value_type;
  typedef std::string reference;

  const VPropStateMap mPropMap;
  const unsigned int mDim;

  RoadmapFromFilePutStateMap(VPropStateMap _propMap, unsigned int _dim)
  : mPropMap{_propMap}
  , mDim{_dim}
  {
  }
};

// Do not allow calling get on this property map
inline std::string
get(const RoadmapFromFilePutStateMap & map,
   const typename RoadmapFromFilePutStateMap::key_type & k)
{
   abort();
}

// Convert string rep. of vector state to Eigen::Vector
inline void
put(const RoadmapFromFilePutStateMap& map,
    const typename RoadmapFromFilePutStateMap::key_type & k,
    const std::string representation)
{
  // Create a vector of length map.mDim, fill it with the values
  // and map the above vector pointer to it
  std::vector<double> v_state_values(map.mDim);
  std::stringstream ss(representation);
  for(unsigned int ui=0; ui < map.mDim; ui++){
    ss >> v_state_values[ui];
  }

  // get() returns the value of the underlying map value type
  // Which is a shared ptr
  get(map.mPropMap, k) = std::make_shared<Eigen::VectorXd>(Eigen::Map<Eigen::VectorXd>(v_state_values.data(), map.mDim));
}


// Take a graph object, the roadmap file name and the state map as input
// And read in the graphml file and populate VProps of each vertex with state
void generateVertices(Graph& _roadmap,
                      VPropStateMap _state_map,
                      std::string _roadmap_filename,
                      unsigned int _dim)
{
  std::ifstream fp;
  fp.open(_roadmap_filename.c_str());

  boost::dynamic_properties props;
  props.property("state",
      RoadmapFromFilePutStateMap(_state_map, _dim));

  boost::read_graphml(fp, _roadmap, props);
}

// Just computes the L2 norm of the difference of the underlying Eigen Vectors
// This is used BOTH by the edge weight map below for computing
// the weight of an edge between two vertex arguments
// AND by the heuristic function which computes the 
// heuristic of the 'this' vertex from the argument vertex
double getL2Weight(const Graph & _roadmap, const Vertex& v1, const Vertex& v2)
{
  // Look up the underlying states and get their L2 diff norm
  double weight{(*(_roadmap[v1].v_state) - *(_roadmap[v2].v_state)).norm()};

  return weight;
}

// Fill in the edge weight map for the graph
// NOTE : This needs some weight function that takes in two Eigen Vectors
// and returns the weight. Above I've defined a default value that just takes the norm
void generateEdges(Graph & _roadmap,
                   EPWeightMap _weight_map)
{
  EdgeIter ei, ei_end;

  // Iterate over edges of roadmap (as defined in graphml file)
  for (boost::tie(ei,ei_end)=edges(_roadmap); ei!=ei_end; ++ei)
  {
    // Lookup the source and target EigenVectors of each edge
    Vertex v1{source(*ei,_roadmap)};
    Vertex v2{target(*ei,_roadmap)};

    // Enter the weight of the edge (as computed via your function)
    // Into the corresponding entry of the edge weight map
    put(_weight_map, *ei, getL2Weight(_roadmap, v1, v2));

    // NOTE - this is a bit non-general (and unnecessary as we never use e_status) but just doing this to 
    // show another easy way of setting edge property values if you know what they are
    // Set the status of each edge to unknown
    _roadmap[*ei].e_status = UNKOWN;

  }
}


// Define the L2 Heuristic for A-star
// Needs the goal vertex and the roadmap as member variables
// To compute the heuristic value for any other
template<class Graph, class CostType>
class l2_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
  l2_heuristic(Vertex _goal_vertex,
               Graph& _roadmap)
  : mGoalVertex{_goal_vertex}
  , mRoadmap{_roadmap}
  {}

  CostType operator()(Vertex u)
  {
    return getL2Weight(mRoadmap, u, mGoalVertex);
  }

private:
  Vertex mGoalVertex;
  Graph& mRoadmap;
};

// Just extends for specificity
class throw_visitor_exception : public std::exception {};

// Define the visitor for A-star search. This simply throws
// an exception when the goal vertex is popped. This ensures control
// returns to the function that is running the search
// This exception is then caught and the higher level code KNOWS
// that the goal vertex has been found (or cannot be found)
class throw_visitor_search
{
public:

  Vertex mVThrow;
  throw_visitor_search(Vertex _v_throw)
  : mVThrow{_v_throw}
  {}
  inline void initialize_vertex(Vertex u, const Graph& g) {}
  inline void discover_vertex(Vertex u, const Graph& g) {}
  
  // Only the examine_vertex method is defined
  // This method is called by the boost Astar code when a
  // vertex is popped (it is called via the visitor object that we will pass to astar_search, this one)
  inline void examine_vertex(Vertex u, const Graph& g)
  {
    if(u == mVThrow) {
      throw throw_visitor_exception();
    }
  }
  inline void examine_edge(Edge e, const Graph& g) {}
  inline void edge_relaxed(Edge e, const Graph & g) {}
  inline void edge_not_relaxed(Edge e, const Graph & g) {}
  inline void black_target(Edge e, const Graph & g) {}
  inline void finish_vertex(Vertex u, const Graph & g) {}
};


// TODO : Currently this assumes all edges are free
// In reality you need to step over embedded configurations in edge
// And return accordingly
bool checkEdgeFree(const Edge& e)
{
  return true;
}


int main(int argc, char* argv[])
{

  po::options_description desc("Arguments for minimal example");

  desc.add_options()
    ("roadmap, r",po::value< std::string >()->required(), "Path to graphml file")
    ("dimensions, d",po::value< unsigned int >()->required(), "Dimensionality of state space")
  ;

  // Read arguments
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  std::string roadmap_filename{vm["roadmap"].as<std::string>()};
  unsigned int n_dims{vm["dimensions"].as<unsigned int>()};

  // Now need to read in the graphml file to a graph
  // First, define the graph
  Graph roadmap_graph;

  // Now call generateVertices to read in the graphml file
  // This will read in the file, assign the underlying state values to vertices
  // (because we call the method with the putter state map) and it will
  // create the edges between vertices (but won't populate edge states)
  generateVertices(roadmap_graph, get(&VProps::v_state,roadmap_graph),
                   roadmap_filename, n_dims);

  // Now call generateEdges to fill in edge properties
  generateEdges(roadmap_graph, get(&EProps::e_weight,roadmap_graph));


  // Almost done! Now your graph is ready, so pick two random states and run
  // astar_search between them
  unsigned int num_verts = static_cast<unsigned int>(boost::num_vertices(roadmap_graph));

  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<unsigned int> uni(0,num_verts-1);

  // Sample random start and goal
  // Vertex is basically unsigned int so can do this
  Vertex start_vertex{uni(rng)};
  Vertex goal_vertex{uni(rng)};
  std::cout<<"Start Vertex - "<<start_vertex<<std::endl<<"Goal Vertex - "<<goal_vertex<<std::endl;

  // Define helper variables needed for astar search
  // The below 4 vars will be POPULATED as a result of a-star search
  // so you can query them later for the search results
  std::map<Vertex,Vertex> startPreds; // Search tree predecessors
  std::map<Vertex,double> startDist; // Cost-to-come from start for popped vertices
  std::map<Vertex,double> startFValue; // F-to-come from start for popped vertices
  std::map<Vertex,boost::default_color_type> colorMap; // Bleh - just boilerplate


  // Use a try-catch block to catch the exception thrown if the goal is visited
  try
  {
    boost::astar_search(
      roadmap_graph, // Obvious
      start_vertex, // Obvious
      l2_heuristic<Graph, double>(goal_vertex, roadmap_graph), // Obvious
      throw_visitor_search(goal_vertex), // Your visitor - just to check when goal popped
      boost::make_assoc_property_map(startPreds), // Boilerplate
      boost::make_assoc_property_map(startFValue), // Boilerplate
      boost::make_assoc_property_map(startDist), // Boilerplate
      boost::get(&EProps::e_weight, roadmap_graph), // The edge property to use for edge cost - we use the e_weight
      boost::get(boost::vertex_index, roadmap_graph), // The map to use for vertex indices
      boost::make_assoc_property_map(colorMap), // Boilerplate
      std::less<double>(), // The lower-than comparator
      boost::closed_plus<double>(std::numeric_limits<double>::max()), // How to add edge weights
      std::numeric_limits<double>::max(), // The value of "infinity" in this context
      double() // The value of "zero"
    );
  }
  catch (const throw_visitor_exception& ex)
  {
    // Do nothing - you just needed the exception to be thrown
  }

  // Check if the cost-to-come for goal is "infinity" - then no path found
  if(startDist[goal_vertex] == std::numeric_limits<double>::max()) {
    throw std::runtime_error("No path from start to goal!");
  }

  // Otherwise, use startPreds to back out a path to the goal
  // And use checkEdge to check the edges along them

  // Walk backwards from goal to start
  Vertex v_walk{goal_vertex};
  double path_weight = 0.0;
  while(v_walk != start_vertex)
  {
    Vertex v_pred{startPreds[v_walk]};

    // Get the edge between vertex and predecessor using boost::edge
    // Which returns a pair of <Edge,bool> where if bool == false then
    // no edge exists between the Vertices in graph
    std::pair<Edge,bool> edge_pair{boost::edge(v_pred, v_walk, roadmap_graph)};

    if(!edge_pair.second) {
      throw std::runtime_error("This really should not happen :P!");
    }

    // This will currently just retur that the edge is free so no worries
    if(!checkEdgeFree(edge_pair.first)){
      std::cout<<"The path is in collision!"<<std::endl;
      return 0;
    }

    // Add to path weight
    path_weight += roadmap_graph[edge_pair.first].e_weight;

    v_walk = v_pred;
  }

  std::cout<<"Weight of path is "<<path_weight<<std::endl;

  return 0;

}




























