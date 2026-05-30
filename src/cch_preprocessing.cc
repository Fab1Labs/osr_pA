#include "osr/cch_preprocessing.h"

#include <iostream>

#include "osr/ways.h"


cch::mip_proc::mip_proc(osr::ways const& w)// std::filesystem::path p, cista::mmap::protection const mode)
  : ways_{w} {}

void cch::mip_proc::test_contraction_order() {
  auto importance_copy_ = ways_.r_->node_importance_;
  importance_copy_.resize(20);
  for (auto const& e : importance_copy_) {std::cout << e << " " << contr_order_[e] << "\n";}
}

// define the contraction order for the preprocessing here
void cch::mip_proc::build_contraction_order() {
  contr_order_.resize(ways_.n_nodes());
  auto i_ = 0;
  for (auto const& node : ways_.r_->node_importance_) {
    contr_order_[node] = osr::node_idx_t{i_};
    ++i_;
  }
}