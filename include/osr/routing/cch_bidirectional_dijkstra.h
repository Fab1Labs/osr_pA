#pragma once

#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace cch {

template <osr::Profile P, bool EarlyTermination = false>
struct bidir_dijkstra {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;

  static constexpr auto const kDebug = false;

  struct get_bucket{
    osr::cost_t operator()(label const& l) {return l.cost();}
  };

  void reset(osr::cost_t const max) {
    pq_f_.clear();
    pq_b_.clear();
    pq_f_.n_buckets(max + 1U);
    pq_b_.n_buckets(max + 1U);
    max_reached_f_ = false;
    max_reached_b_ = false;
    cost_f_.clear();
    cost_b_.clear();
  }

  // void add_start(osr::ways const& w, label const l) {

  // }

  osr::location start_loc_;
  osr::location end_loc_;
  osr::dial<label, get_bucket> pq_f_{get_bucket{}};
  osr::dial<label, get_bucket> pq_b_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_f_;
  ankerl::unordered_dense::map<key, entry, hash> cost_b_;
  osr::cost_t mu_;
  bool max_reached_f_{};
  bool max_reached_b_{};
};
} // namespace cch