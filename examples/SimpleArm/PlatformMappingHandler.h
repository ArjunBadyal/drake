#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
//#include "drake/geometry/query_object.h"


namespace drake {
using systems::LeafSystem;
using systems::InputPort;
using systems::OutputPort;
using systems::BasicVector;
using systems::Context;
using systems::EventStatus;
namespace examples {
namespace multibody {
namespace SimpleArm {


//template <typename T>

class PMH final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PMH)

  PMH();

  const systems::InputPort<double>& PMH_input_port() const {
    return this->get_input_port(PMH_input_port_);
  }

 private:
  /*void CalcOutput(const Context<double>& context, BasicVector<double>* output) const;
   */
  int PMH_input_port_;
  // event handler
  /*void CalcOutput(const Context<double>& context,
                  Eigen::VectorBlock<VectorX<double>>* output) const;*/

  void CalcOutput(const Context<double>& context,
                  BasicVector<double>* output) const;

};





}  // namespace sensors

            }  // namespace systems
        }  // namespace drake
    }
