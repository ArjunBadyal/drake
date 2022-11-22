//
// Created by arjunbadyal on 14/11/2022.
//

#pragma once
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
using systems::LeafSystem;
using systems::InputPort;
using systems::OutputPort;
using systems::BasicVector;
using systems::Context;
using systems::SystemTypeTag;
using systems::kAbstractValued;
using systems::kVectorValued;

namespace examples {
namespace multibody {
namespace SimpleArm {

class PMHandler final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PMHandler)

  PMHandler();

  ~PMHandler() final = default;

  const InputPort<double>& distance_input_port() const;

 private:
  void CalcOutput(const Context<double>& context,
                  Eigen::VectorBlock<VectorX<double>>* output) const;
};

}
}
}
}