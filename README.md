# BGL_MINIMAL_EXAMPLE

This repository provides a minimal working example of running [A* graph search](https://brilliant.org/wiki/a-star-search/) with [Boost Graph Library](https://www.boost.org/doc/libs/1_66_0/libs/graph/doc/index.html). There are more general examples of this use case [[1](https://www.boost.org/doc/libs/1_54_0/libs/graph/example/astar-cities.cpp),[2](https://github.com/wpm/Astar-Maze-Solver)], but this example focuses on the use case for [roadmap-based path planning](http://planning.cs.uiuc.edu/node240.html). 

Here, each vertex has an attribute which is the [configuration space](http://planning.cs.uiuc.edu/node123.html) point that it represents, and each edge has attributes for the edge weight (typically some distance function between vertices) and the edge status (whether collision-free or not).

The following behaviour is exemplified here :

- Loading a Boost `.graphml` file that defines the configuration that each vertex represents, and the edges connecting vertices. This is sometimes called an explicit roadmap (graph).
- Defining the `struct` types that represent the attributes of the vertices and edges of the graph.
- Creating a `Boost Graph` (specifically an undirected graph represented as an adjacency list), as well as the [property maps](https://www.boost.org/doc/libs/1_55_0/libs/graph/doc/using_property_maps.html) that map the vertices and edges to their attributes.
- Defining a [heuristic function](https://www.boost.org/doc/libs/1_46_0/libs/graph/doc/astar_search.html) for A* search that depends on the underlying configurations of the vertices (as they typically should).
- Running `boost::astar_search` in its most general form with all the relevant arguments, to obtain the shortest path between two randomly chosen vertices.

## Dependencies

This has been tested on Ubuntu 14.04 and 16.04 as well as MacOS Sierra (with some particular tweaks mentioned below). Following are the dependencies:

- GCC (>=4.9)
- CMake
- [Boost](https://stackoverflow.com/questions/12578499/how-to-install-boost-on-ubuntu)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download)

*A Note on MacOS* - What worked for me was to replace `${Boost_INCLUDE_DIRS}` with `${Boost_INCLUDE_DIR}` and `${EIGEN3_INCLUDE_DIRS}` with `${EIGEN3_INCLUDE_DIR}` in `CMakeLists.txt`.


## Usage

To run, clone the repo and do the following inside the top-level directory of the repo:

```
mkdir build && cd build
cmake ..
make
./example --r ../data/halton_2d_withedges.graphml --d 2
```

## Additional Comments

The `data/` folder has an example `.graphml` file where the vertices are sampled on a 2D unit grid. The roadmap has 30 vertices and the configurations are generated from a 2D [Halton sequence](https://en.wikipedia.org/wiki/Halton_sequence), and edges connect vertices with mutual Euclidean distance less than some threshold. The graphs themselves were generated with [NetworkX](https://networkx.github.io/documentation/networkx-1.10/reference/readwrite.graphml.html) where the configurations are specified as vertex attributes.

It is not best practice to place everything in one `.cpp` file, especially when there are reusable modules, but I have done so for the purposes of this minimal example to make it easier to follow. Finally, I have put in some `NOTE:` tags in the commands to highlight important points about usage. 