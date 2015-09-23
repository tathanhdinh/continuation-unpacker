#include "../tinyformat.h"
#include "graph.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdexcept>

//extern p_instructions_t trace;
extern map_address_instruction_t cached_ins_at_addr;

static tr_graph_t internal_graph;

static bb_graph_t internal_bb_cfg;
static bb_vertex_desc_t root_bb_cfg_desc = bb_graph_t::null_vertex();

static bb_graph_t internal_bb_tree;
static bb_vertex_desc_t root_bb_tree_desc = bb_graph_t::null_vertex();

static auto find_vertex (tr_vertex_t vertex_value) -> tr_vertex_iter_t 
{
  auto first_vertex_iter = tr_vertex_iter_t();
  auto last_vertex_iter  = tr_vertex_iter_t();
  std::tie(first_vertex_iter, last_vertex_iter) = boost::vertices(internal_graph);

  auto found_vertex_iter = last_vertex_iter;
  for (found_vertex_iter = first_vertex_iter; found_vertex_iter != last_vertex_iter; ++found_vertex_iter) {
    if (internal_graph[*found_vertex_iter] == vertex_value) break;
  }

  return found_vertex_iter;
}


auto construct_graph_from_trace (const p_instructions_t& trace) -> void
{
  auto prev_vertex_desc = tr_graph_t::null_vertex();
  for (const auto& inst : trace) {
    auto ins_addr = inst->address;

    auto curr_vertex_iter = find_vertex(ins_addr);
    auto curr_vertex_desc = tr_vertex_desc_t();

    if (curr_vertex_iter == std::get<1>(boost::vertices(internal_graph))) {
      curr_vertex_desc = boost::add_vertex(ins_addr, internal_graph);
    }
    else curr_vertex_desc = *curr_vertex_iter;

    if (prev_vertex_desc != tr_graph_t::null_vertex()) {
      if (!std::get<1>(boost::edge(prev_vertex_desc, curr_vertex_desc, internal_graph))) {
        boost::add_edge(prev_vertex_desc, curr_vertex_desc, internal_graph);
      }
    }

    prev_vertex_desc = curr_vertex_desc;
  }

  return;
}


auto is_loopback_vertex(bb_vertex_desc_t vertex, const bb_graph_t& graph) -> bool
{
  auto first_out_edge_iter = bb_graph_t::out_edge_iterator();
  auto last_out_edge_iter  = bb_graph_t::out_edge_iterator();

  std::tie(first_out_edge_iter, last_out_edge_iter) = boost::out_edges(vertex, graph);

  return std::any_of(first_out_edge_iter, last_out_edge_iter, [&vertex, &graph](bb_edge_desc_t edge_desc)
  {
    return (boost::target(edge_desc, graph) == vertex);
  });
}


template<bool cfg_or_tree>
static auto is_first_bb (bb_vertex_desc_t vertex_desc) -> bool
{
//  return (std::get<BB_ADDRESSES>(internal_bb_graph[vertex_desc]).front() == trace.front()->address);
  if (cfg_or_tree) return (root_bb_cfg_desc == vertex_desc);
  else return (root_bb_tree_desc == vertex_desc);
}


template<bool cfg_or_tree>
static auto is_pivot_vertex (bb_vertex_desc_t vertex_desc) -> bool
{
  const auto& graph = cfg_or_tree ? internal_bb_cfg : internal_bb_tree;

  if (!is_loopback_vertex(vertex_desc, graph)) {
    auto in_degree = boost::in_degree(vertex_desc, graph);
    auto out_degree = boost::out_degree(vertex_desc, graph);

    if (out_degree == 1) {
      auto out_edge_iter = bb_graph_t::out_edge_iterator();
      std::tie(out_edge_iter, std::ignore) = boost::out_edges(vertex_desc, graph);

      auto next_vertex = boost::target(*out_edge_iter, graph);

      if ((boost::in_degree(next_vertex, graph) == 1) &&
          !is_loopback_vertex(next_vertex, graph) && !is_first_bb<cfg_or_tree>(next_vertex)) {

        if ((in_degree == 0) || (in_degree >= 2)) { // pivot found
          return true;
        }
        else {
          auto in_edge_iter = bb_graph_t::in_edge_iterator();
          std::tie(in_edge_iter, std::ignore) = boost::in_edges(vertex_desc, graph);

          auto prev_vertex = boost::source(*in_edge_iter, graph);
          if (boost::out_degree(prev_vertex, graph) >= 2) // pivot found
            return (is_loopback_vertex(prev_vertex, graph) || (boost::out_degree(prev_vertex, graph) >= 2));
          else return is_first_bb<cfg_or_tree>(vertex_desc);
        }
      }
      else return false;
    }
    else return false;
  }
  return false;
}

template<bool cfg_or_tree>
static auto find_pivot_vertex () -> bb_vertex_desc_t
{
  auto first_vertex_iter = bb_vertex_iter_t();
  auto last_vertex_iter  = bb_vertex_iter_t();

  if (cfg_or_tree) std::tie(first_vertex_iter, last_vertex_iter) = boost::vertices(internal_bb_cfg);
  else std::tie(first_vertex_iter, last_vertex_iter) = boost::vertices(internal_bb_tree);

  auto found_vertex_iter = std::find_if(first_vertex_iter, last_vertex_iter, is_pivot_vertex<cfg_or_tree>);
  if (found_vertex_iter != last_vertex_iter) return *found_vertex_iter;
  else return bb_graph_t::null_vertex();
}


static auto compress_graph_from_pivot_vertex (bb_vertex_desc_t pivot_vertex, bb_graph_t& graph) -> bb_vertex_desc_t
{
  auto out_edge_iter = bb_graph_t::out_edge_iterator();
  std::tie(out_edge_iter, std::ignore) = boost::out_edges(pivot_vertex, graph);

  auto next_vertex = boost::target(*out_edge_iter, graph);

  boost::remove_out_edge_if(pivot_vertex, [](bb_edge_desc_t edge_desc) { return true; }, graph);

  std::get<BB_ADDRESSES>(graph[pivot_vertex]).insert(
        std::end(std::get<BB_ADDRESSES>(graph[pivot_vertex])),
        std::begin(std::get<BB_ADDRESSES>(graph[next_vertex])),
        std::end(std::get<BB_ADDRESSES>(graph[next_vertex]))
        );

  auto last_out_edge_iter = bb_graph_t::out_edge_iterator();
  std::tie(out_edge_iter, last_out_edge_iter) = boost::out_edges(next_vertex, graph);

  for (auto edge_iter = out_edge_iter; edge_iter != last_out_edge_iter; ++edge_iter) {
    auto next_next_vertex = boost::target(*edge_iter, graph);
    boost::add_edge(pivot_vertex, next_next_vertex, graph);
//    boost::remove_edge(edge_iter, internal_bb_graph);
  }
  boost::remove_out_edge_if(next_vertex, [](bb_edge_desc_t edge_desc) { return true; }, graph);

  boost::remove_vertex(next_vertex, graph);
  return pivot_vertex;
}


//static auto linear_node_order = int32_t{0};
static auto linear_visited_bbs = std::vector<bb_vertex_desc_t>{};
struct node_numbering_visitor : public boost::default_bfs_visitor
{
//  static std::vector<bb_vertex_desc_t> linear_visited_bbs;

  void discover_vertex(bb_vertex_desc_t vertex_desc, const bb_graph_t& graph)
  {
    linear_visited_bbs.push_back(vertex_desc);
    return;
  }
};


static auto current_order = int32_t{0};
static auto unnumbered_vertices = std::queue<bb_vertex_desc_t>{};
static auto numbering_from_vertex (bb_vertex_desc_t current_vertex, bb_graph_t& graph) -> void
{
  std::get<BB_ORDER>(graph[current_vertex]) = current_order;
  ++current_order;

  auto first_out_edge_iter = bb_graph_t::out_edge_iterator{};
  auto last_out_edge_iter = bb_graph_t::out_edge_iterator{};
  std::tie(first_out_edge_iter, last_out_edge_iter) = boost::out_edges(current_vertex, graph);

  auto next_unnumbered_vertices = std::vector<bb_vertex_desc_t>{};
  for (auto out_edge_iter = first_out_edge_iter; out_edge_iter != last_out_edge_iter; ++out_edge_iter) {
    auto next_vertex = boost::target(*out_edge_iter, graph);
    if (std::get<BB_ORDER>(graph[next_vertex]) == -1) {
      next_unnumbered_vertices.push_back(next_vertex);
    }
  }

  // deciding order of traversing
  std::stable_sort(
        std::begin(next_unnumbered_vertices), std::end(next_unnumbered_vertices), [&graph](bb_vertex_desc_t va, bb_vertex_desc_t vb)
  {
    return (std::get<BB_ADDRESSES>(graph[va]).front() < std::get<BB_ADDRESSES>(graph[vb]).front());
  });

  for (auto next_vertex_desc : next_unnumbered_vertices) {
//    numbering_from_vertex(next_vertex_desc);
    unnumbered_vertices.push(next_vertex_desc);
  }

  return;
}


template<bool cfg_or_tree>
static auto numbering_bb_graph (bb_graph_t& graph) -> void
{
  auto first_bb_iter = bb_vertex_iter_t{};
  auto last_bb_iter  = bb_vertex_iter_t{};
  std::tie(first_bb_iter, last_bb_iter) = boost::vertices(graph);

//  auto bb_iter = std::find_if(first_bb_iter, last_bb_iter, [](bb_vertex_desc_t bb_vertex_desc) {
//    return (boost::in_degree(bb_vertex_desc, internal_bb_graph) == 0);
//  });
  auto bb_iter = std::find_if(first_bb_iter, last_bb_iter, is_first_bb<cfg_or_tree>);
  assert(bb_iter != last_bb_iter);

  // deterministic BFS traversing
  auto first_bb_vertex_desc = *bb_iter;
  unnumbered_vertices.push(first_bb_vertex_desc);
  while (!unnumbered_vertices.empty()) {
    auto considered_vertex = unnumbered_vertices.front();
    unnumbered_vertices.pop();
    numbering_from_vertex(considered_vertex, graph);
  }
//  numbering_from_vertex(first_bb_vertex_desc);

//  auto begin_bb_iter = bb_vertex_iter_t{};
//  auto end_bb_iter = bb_vertex_iter_t{};

//  auto start_bb_desc = bb_vertex_desc_t{};

//  assert(boost::num_vertices(internal_bb_graph) > 0);

//  std::tie(begin_bb_iter, end_bb_iter) = boost::vertices(internal_bb_graph);

//  auto visitor = node_numbering_visitor();
//  linear_visited_bbs.clear();
//  boost::breadth_first_search(internal_bb_graph, start_bb_desc, boost::visitor(visitor));

//  auto linear_order = 0;
//  tfm::printfln("size of graph %d", linear_visited_bbs.size());
//  for (auto& bb : linear_visited_bbs) {
//    std::get<BB_ORDER>(internal_bb_graph[bb]) = linear_order;
//    ++linear_order;
//  }

  return;
}


//static auto get_tr_vertex_desc (tr_vertex_t addr) -> tr_vertex_desc_t
//{
//  auto first_vertex_iter = tr_vertex_iter_t();
//  auto last_vertex_iter = tr_vertex_iter_t();

//  std::tie(first_vertex_iter, last_vertex_iter) = boost::vertices(internal_graph);

//  for (auto vertex_iter = first_vertex_iter; vertex_iter != last_vertex_iter; ++vertex_iter) {
//    if (internal_graph[*vertex_iter] == addr) return *vertex_iter;
//  }
//  return tr_graph_t::null_vertex();
//}


static auto get_bb_vertex_desc_in_cfg (tr_vertex_t addr) -> bb_vertex_desc_t
{
  auto first_vertex_iter = bb_vertex_iter_t();
  auto last_vertex_iter = bb_vertex_iter_t();

  std::tie(first_vertex_iter, last_vertex_iter) = boost::vertices(internal_bb_cfg);
  for (auto vertex_iter = first_vertex_iter; vertex_iter != last_vertex_iter; ++vertex_iter) {
    if (std::get<1>(internal_bb_cfg[*vertex_iter]).front() == addr) return *vertex_iter;
  }
  return bb_graph_t::null_vertex();
}


//auto construct_bb_trace (const p_instructions_t& trace) -> std::vector<bb_vertex_desc_t>
//{
//  auto bb_trace = std::vector<bb_vertex_desc_t>{};

//  auto ins_iter = std::begin(trace);
//  auto last_ins_iter = std::end(trace);

//  auto first_bb_iter = bb_vertex_iter_t{};
//  auto last_bb_iter = bb_vertex_iter_t{};
//  std::tie(first_bb_iter, last_bb_iter) = boost::vertices(internal_bb_graph);

////  auto bb_iter = std::find_if(first_bb_iter, last_bb_iter, [](bb_vertex_desc_t bb_vertex_desc) {
////    return (boost::in_degree(bb_vertex_desc, internal_bb_graph) == 0);
////  });
//  auto bb_iter = std::find_if(first_bb_iter, last_bb_iter, is_first_bb);
//  assert(bb_iter != last_bb_iter);

//  auto bb_vertex_desc = *bb_iter;

//  while (ins_iter != last_ins_iter) {
//    for (auto & ins_addr : std::get<BB_ADDRESSES>(internal_bb_graph[bb_vertex_desc])) {
//      assert(ins_addr == (*ins_iter)->address);
//      ++ins_iter;
//    }

//    bb_trace.push_back(bb_vertex_desc);

//    if (ins_iter != last_ins_iter) {
//      auto first_out_edge_iter = bb_graph_t::out_edge_iterator{};
//      auto last_out_edge_iter = bb_graph_t::out_edge_iterator{};
//      std::tie(first_out_edge_iter, last_out_edge_iter) = boost::out_edges(bb_vertex_desc, internal_bb_graph);

//      auto next_out_edge_iter = std::find_if(first_out_edge_iter, last_out_edge_iter, [&](bb_edge_desc_t bb_edge_desc) {
//        auto target_vertex_desc = boost::target(bb_edge_desc, internal_bb_graph);
//        return ((*ins_iter)->address == std::get<BB_ADDRESSES>(internal_bb_graph[target_vertex_desc]).front());
//      });

//      assert(next_out_edge_iter != last_out_edge_iter);
//      bb_vertex_desc = boost::target(*next_out_edge_iter, internal_bb_graph);
//    }
//  }

//  return bb_trace;
//}


auto cap_save_trace_to_dot_file (const p_instructions_t& trace, const std::string& filename) -> void
{
  auto write_vertex = [](std::ostream& label, tr_vertex_desc_t vertex_desc) -> void {
    tfm::format(label, "[label=\"0x%x:%s\"]", internal_graph[vertex_desc],
                cached_ins_at_addr[internal_graph[vertex_desc]]->disassemble);
    return;
  };

  auto write_edge = [](std::ostream& label, tr_edge_desc_t edge_desc) -> void {
    tfm::format(label, "[label=\"\"]");
    return;
  };

  if (trace.size() > 0) {
    construct_graph_from_trace(trace);

    std::ofstream output_file(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (output_file.is_open()) {

      boost::write_graphviz(output_file, internal_graph,
                            std::bind(write_vertex, std::placeholders::_1, std::placeholders::_2),
                            std::bind(write_edge, std::placeholders::_1, std::placeholders::_2));
      output_file.close();
    }
    else {
      tfm::printfln("cannot save trace to dot file %s", filename);
    }
  }
  else tfm::printfln("trace is empty, graph contruction is omitted");

  return;
}


template<bool cfg_or_tree>
static auto write_graph_vertex (std::ostream& label, bb_vertex_desc_t vertex_desc) -> void
{
  auto bb_graph = cfg_or_tree ? internal_bb_cfg : internal_bb_tree;

  if (is_first_bb<cfg_or_tree>(vertex_desc)) {
      tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=cornflowerblue,label=\"");
  }
  else if (boost::out_degree(vertex_desc, bb_graph) == 0) {
    tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=gainsboro,label=\"");
  }
  else if (boost::in_degree(vertex_desc, bb_graph) > 2) {
    tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=darkorchid1,label=\"");
  }
  else if (boost::out_degree(vertex_desc, bb_graph) > 2) {
    tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=darkgoldenrod1,label=\"");
  }
  else tfm::format(label, "[shape=box,style=rounded,label=\"");

  tfm::format(label, "%d\n", std::get<BB_ORDER>(bb_graph[vertex_desc]));
  for (const auto& addr : std::get<BB_ADDRESSES>(bb_graph[vertex_desc])) {
    /*if (std::addressof(addr) == std::addressof(internal_bb_graph[vertex_desc].back())) {
      tfm::format(label, "%-12s %-s", StringFromAddrint(addr), cached_ins_at_addr[addr]->disassemble);
    }
    else*/
    tfm::format(label, "0x%-12x %-s\\l", addr, cached_ins_at_addr[addr]->disassemble);
  }
  tfm::format(label, "\",fontname=\"Inconsolata\",fontsize=10.0]");

  return;  
}


template<bool cfg_or_tree>
static auto write_graph_thumbnail_vertex (std::ostream& label, bb_vertex_desc_t vertex_desc) -> void
{
  auto bb_graph = cfg_or_tree ? internal_bb_cfg : internal_bb_tree;

  if (is_first_bb<cfg_or_tree>(vertex_desc)) {
      tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=cornflowerblue,label=\"");
  }
  else if (boost::out_degree(vertex_desc, bb_graph) == 0) {
    tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=gainsboro,label=\"");
  }
  else if (boost::in_degree(vertex_desc, bb_graph) > 2) {
    tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=darkorchid1,label=\"");
  }
  else if (boost::out_degree(vertex_desc, bb_graph) > 2) {
    tfm::format(label, "[shape=box,style=\"filled,rounded\",fillcolor=darkgoldenrod1,label=\"");
  }
  else tfm::format(label, "[shape=box,style=rounded,label=\"");

  tfm::format(label, "%d", std::get<BB_ORDER>(bb_graph[vertex_desc]));
  tfm::format(label, "\",fontname=\"Inconsolata\",fontsize=10.0]");
  return;
}


static auto write_cfg_edge (std::ostream& label, bb_edge_desc_t edge_desc) -> void 
{
  tfm::format(label, "[label=\"\"]");
  return;
}

/* ===================================== exported functions ===================================== */

auto add_trace_into_basic_block_cfg (const p_instructions_t& trace) -> void
{
  if (trace.empty()) throw 0;

  if (boost::num_vertices(internal_bb_cfg) == 0) {
    auto ins_addr = trace.front()->address;
    root_bb_cfg_desc = boost::add_vertex(bb_vertex_t(-1, tr_vertices_t{ins_addr}), internal_bb_cfg);
  }

  if (internal_bb_cfg[root_bb_cfg_desc].second.front() != trace.front()->address)
    throw std::logic_error("the first instruction does not match");

  auto prev_bb_desc = bb_graph_t::null_vertex();
  for (const auto& inst : trace) {
    auto ins_addr = inst->address;

    auto curr_bb_desc = get_bb_vertex_desc_in_cfg(ins_addr);
    if (curr_bb_desc == bb_graph_t::null_vertex()) {
      curr_bb_desc = boost::add_vertex(bb_vertex_t(-1, tr_vertices_t{ins_addr}), internal_bb_cfg);
    }

    if (prev_bb_desc != bb_graph_t::null_vertex()) {
      if (!std::get<1>(boost::edge(prev_bb_desc, curr_bb_desc, internal_bb_cfg))) {
        boost::add_edge(prev_bb_desc, curr_bb_desc, internal_bb_cfg);
      }
    }

    prev_bb_desc = curr_bb_desc;
  }

  return;
}


template<bool cfg_or_tree>
auto construct_basic_block_graph () -> void
{
  auto& graph = cfg_or_tree ? internal_bb_cfg : internal_bb_tree;

  boost::progress_display compress_progress(boost::num_vertices(graph));
  auto pivot_vertex_desc = bb_graph_t::null_vertex();
  do {
    ++compress_progress;
    if ((pivot_vertex_desc == bb_graph_t::null_vertex()) || !is_pivot_vertex<cfg_or_tree>(pivot_vertex_desc))
      pivot_vertex_desc = find_pivot_vertex<cfg_or_tree>();

    if (pivot_vertex_desc != bb_graph_t::null_vertex()) {
      pivot_vertex_desc = compress_graph_from_pivot_vertex(pivot_vertex_desc, graph);
    }
    else break;
  }
  while (true);

  numbering_bb_graph<cfg_or_tree>(graph);

  return;
}
template void construct_basic_block_graph<true>();
template void construct_basic_block_graph<false>();

template<bool cfg_or_tree>
auto save_basic_block_graph_to_dot_file (const std::string& filename) -> void
{
  const auto& graph = cfg_or_tree ? internal_bb_cfg : internal_bb_tree;

  std::ofstream output_file(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

  boost::write_graphviz(output_file, graph,
                        std::bind(write_graph_thumbnail_vertex<cfg_or_tree>, std::placeholders::_1, std::placeholders::_2),
                        std::bind(write_cfg_edge, std::placeholders::_1, std::placeholders::_2));
  output_file.close();
  return;
}
template void save_basic_block_graph_to_dot_file<true>(const std::string& filename);
template void save_basic_block_graph_to_dot_file<false>(const std::string& filename);

//auto save_basic_block_trace_to_file (const p_instructions_t& trace, const std::string& filename) -> void
//{
//  std::ofstream output_file(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
//  auto bb_trace = construct_bb_trace(trace);

//  for (auto & bb_vertex_desc : bb_trace) {
//    tfm::format(output_file, "%d ", std::get<BB_ORDER>(internal_bb_graph[bb_vertex_desc]));
//  }

//  output_file.close();

//  return;
//}


auto add_trace_into_basic_block_tree (const p_instructions_t& trace) -> void
{
  if (trace.empty()) return;

  if (boost::num_vertices(internal_bb_tree) == 0) {
    auto ins_addr = trace.front()->address;
    root_bb_tree_desc = boost::add_vertex(bb_vertex_t(-1, tr_vertices_t{ins_addr}), internal_bb_tree);
  }

  if (std::get<1>(internal_bb_tree[root_bb_tree_desc]).front() != trace.front()->address)
    throw std::logic_error("the first instruction does not match");

  auto current_bb_desc = root_bb_tree_desc;
  auto next_bb_desc = bb_graph_t::null_vertex();

  auto inst_iter = std::begin(trace); ++inst_iter;
  auto last_inst_iter = std::end(trace);

  for (; inst_iter != last_inst_iter; ++inst_iter) {
    auto inst_addr = (*inst_iter)->address;

    auto first_out_edge_iter = bb_graph_t::out_edge_iterator{};
    auto last_out_edge_iter = bb_graph_t::out_edge_iterator{};

    std::tie(first_out_edge_iter, last_out_edge_iter) = boost::out_edges(current_bb_desc, internal_bb_tree);
    auto next_out_edge_iter = std::find_if(first_out_edge_iter, last_out_edge_iter, [&](bb_edge_desc_t edge_desc)
    {
      auto target_vertex_desc = boost::target(edge_desc, internal_bb_tree);
      return (std::get<1>(internal_bb_tree[target_vertex_desc]).front() == inst_addr);
    });

    if (next_out_edge_iter != last_out_edge_iter) {
      next_bb_desc = boost::target(*next_out_edge_iter, internal_bb_tree);
    }
    else {
      next_bb_desc = boost::add_vertex(bb_vertex_t(-1, tr_vertices_t{inst_addr}), internal_bb_tree);
      boost::add_edge(current_bb_desc, next_bb_desc, internal_bb_tree);
    }

    current_bb_desc = next_bb_desc;
  }

  return;
}
