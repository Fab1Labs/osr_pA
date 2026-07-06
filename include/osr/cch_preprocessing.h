#pragma once

#include <numeric>
#include <algorithm>
#include <utility>

#include "cista/containers/vector.h"
#include "cista/strong.h"
#include "cista/io.h"
#include "utl/enumerate.h"

#include "osr/ways.h"
#include "osr/types.h"

namespace cch {

struct neighbor {
  osr::node_idx_t node_;
  osr::way_idx_t edge_;
  osr::direction dir_;
};

struct neighborhood {

  neighborhood(osr::node_idx_t const& node, std::uint32_t const& rank);
  void sort_neighbors();
  void filter_higher_neighbors();
  bool is_neighbor(osr::node_idx_t const&);
  void concatenate(neighborhood const&);

  osr::node_idx_t const node_;
  std::uint32_t const rank_;
  osr::vec<
    std::tuple<osr::node_idx_t, // neighbor_node
    std::uint32_t, //neighbor node rank
    bool, // shortcut neighbor?
    osr::node_idx_t, // shortcut over this node
    osr::way_idx_t, // way to via node
    osr::way_idx_t // way from via node
  >> neighbors_;
};

struct mip_proc {
  explicit mip_proc(osr::ways&);

  void build_contraction_order();
  bool check_importance(osr::node_idx_t const&, osr::node_idx_t const&);
  bool is_in(osr::vec<std::tuple<osr::node_idx_t, std::uint32_t, bool, osr::node_idx_t, osr::way_idx_t, osr::way_idx_t>> const&, osr::node_idx_t const&);
  void init_neighborhoods();
  void contract_nodes();
  void write_shortcuts(cista::mmap::protection);
  void basic_customization();

  cista::mmap mm(char const* file, cista::mmap::protection mode) {
    return cista::mmap{(ways_.p_ / file).generic_string().c_str(), mode};
  }

  osr::ways& ways_;
  osr::vec<osr::node_idx_t> contr_order_;
  osr::vec<neighborhood> all_neighbors_;
  osr::vec<std::uint32_t> elimination_tree_;
};

} //namespace cch