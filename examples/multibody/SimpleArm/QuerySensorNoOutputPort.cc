#include "drake/examples/multibody/SimpleArm/QuerySensorNoOutputPort.h"

#include "drake/geometry/query_object.h"

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



                    QuerySensorNoOutputPort::QuerySensorNoOutputPort(): LeafSystem<double>() {

                        // Declare depth input port.
                       
                        query_sensor_input_port_ =
                            this->DeclareAbstractInputPort("query_object", Value<drake::geometry::QueryObject<double>>())
                                .get_index();

                        // Declare measurement output port.
                        this->DeclareVectorOutputPort("depth2", kVectorValued,
                                                      &QuerySensor::CalcOutput);
                    }

                 /*   const InputPort<double>& QuerySensor::get_depth_input_port() const {
                      return this->get_input_port(0);
                    }*/



                    void QuerySensor::CalcOutput(const systems::Context<double> &context,
                                                    BasicVector<double> *output) const {
                        const auto params = dynamic_cast<const QuerySensor *>(
                                &context.get_numeric_parameter(0));
                        DRAKE_DEMAND(params != nullptr);
                        const auto &depth = query_sensor_input_port().Eval(context);
                        auto val = depth(0);
                        drake::log()->info(val);
                        drake::log()->info("hello");

                        //drake::geometry::QueryObject<double>::ComputeSignedDistance()


                        auto measurement = output->get_mutable_value();

                        measurement(0) = val;

                    }

                }  // namespace sensors
            }  // namespace systems
        }  // namespace drake
    }

