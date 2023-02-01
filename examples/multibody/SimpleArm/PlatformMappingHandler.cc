#include "drake/examples/multibody/SimpleArm/PlatformMappingHandler.h"

//#include "drake/geometry/query_object.h"


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



PMH::PMH(): LeafSystem<double>() {

  PMH_input_port_ =
      this->DeclareVectorInputPort("PMH_Input", 1).get_index();

  this->DeclareVectorOutputPort("VelCmd",BasicVector<double>(4), &PMH::CalcOutput);

}



void PMH::CalcOutput(const Context<double>& context,
                          BasicVector<double>* output) const{

  BasicVector<double> comm(4);


  //BasicVector<double> comm(2);
  auto val = PMH_input_port().Eval(context);
  double distance = val[0];
  //drake::log()->info(distance);
comm[1] = Operation(distance,distance, context.get_time());
  //drake::log()->info(context.get_time() - t0);
comm[0] = 0.0;
comm[2] = 0.0;
comm[3] = 0.0;

//for picking the box to investigate contact and collision
/*if (context.get_time() < 1.59 && context.get_time() > 1.57)
{
  comm[2] = 0.1;
  comm[3] = -0.1;
}*/
  output->get_mutable_value()  = comm.get_value();
}

}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

