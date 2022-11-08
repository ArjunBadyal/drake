#include "drake/examples/multibody/SimpleArm/QueryObject.h"

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/random.h"

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

                template<typename T>
                QueryObject<T>::QueryObject(): LeafSystem<T>(SystemTypeTag<QueryObject>{}) {

                    // Declare depth input port.
                    this->DeclareInputPort("depth", kAbstractValued);

                    // Declare measurement output port.
                    this->DeclareVectorOutputPort("depth", kVectorValued,
                                                  &QueryObject<T>::CalcOutput);
                }

                template <typename T>
                template <typename U>
                QueryObject<T>::QueryObject(const QueryObject<U>& )
                        : QueryObject<T>() {}

                template<typename T>
                void QueryObject<T>::CalcOutput(const systems::Context<T> &context,
                                                BasicVector<T> *output) const {
                    const auto params = dynamic_cast<const QueryObject<T> *>(
                            &context.get_numeric_parameter(0));
                    DRAKE_DEMAND(params != nullptr);
                    const auto &depth = get_depth_input_port().Eval(context);
                    auto val = depth.ComputeSignedDistancePairwiseClosestPoints();
                    drake::log()->info(val);


                    auto measurement = output->get_mutable_value();

                    measurement(0) = depth;

                }

            }  // namespace sensors
        }  // namespace systems
    }  // namespace drake
}

