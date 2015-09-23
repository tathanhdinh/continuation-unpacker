#ifndef GRAPH_H
#define GRAPH_H

#include "../type/instruction.h"

#include <vector>
#include <queue>
#include <list>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cinttypes>
#include <cstdint>


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/progress.hpp>

using tr_vertex_t = uint32_t;
using tr_vertices_t = std::vector<tr_vertex_t>;
using tr_graph_t = boost::adjacency_list<boost::listS,
                                         boost::vecS,
                                         boost::bidirectionalS,
                                         tr_vertex_t>;
using tr_vertex_desc_t = tr_graph_t::vertex_descriptor;
using tr_edge_desc_t = tr_graph_t::edge_descriptor;
using tr_vertex_iter_t = tr_graph_t::vertex_iterator;
using tr_edge_iter_t = tr_graph_t::edge_iterator;

using bb_vertex_t = std::pair<int32_t, tr_vertices_t>;
enum
{
  BB_ORDER = 0,
  BB_ADDRESSES = 1
};
using bb_graph_t = boost::adjacency_list<boost::listS,
                                         boost::vecS,
                                         boost::bidirectionalS,
                                         bb_vertex_t>;
using bb_vertex_desc_t = bb_graph_t::vertex_descriptor;
using bb_edge_desc_t = bb_graph_t::edge_descriptor;
using bb_vertex_iter_t = bb_graph_t::vertex_iterator;
using bb_edge_iter_t = bb_graph_t::edge_iterator;

#endif // GRAPH_H

