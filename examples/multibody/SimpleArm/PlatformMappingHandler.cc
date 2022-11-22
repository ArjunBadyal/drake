#include "drake/examples/multibody/SimpleArm/PlatformMappingHandler.h"

#include "drake/geometry/query_object.h"

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/random.h"
#include "PlatformMapping.h"

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



PMHandler::PMHandler(): LeafSystem<double>() {

  this->DeclareVectorOutputPort("VelCmd", &PMHandler::CalcOutput);
}

const InputPort<double>& PMHandler::distance_input_port() const {
  return this->get_input_port(0);
}



void PMHandler::CalcOutput(const Context<double> &context, Eigen::VectorBlock<VectorX<double>>* output) const{

  Eigen::Vector2d command;

  if(input == "PrePick") {
    command << PrePick(context.get_time() - t0), 0;
  }
  else if(input == "PrePlace"){
    command << PrePlace(context.get_time() - t0), 0;
  }
  else if(input == "Return") {
    command << Ret(context.get_time() - t0), 0;
  }
  //drake::log()->info(context.get_time() - t0);
  output->segment(0, 2) = command;
}

}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

