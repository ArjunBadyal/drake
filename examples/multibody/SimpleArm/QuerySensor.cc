#include "drake/examples/multibody/SimpleArm/QuerySensor.h"

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
    using systems::EventStatus;
    using geometry::QueryObject;

    namespace examples {
        namespace multibody {
            namespace SimpleArm {


//template <typename T>
                    QuerySensor::QuerySensor(): LeafSystem<double>() {
                        // Declare depth input port.
                        query_sensor_input_port_ =
                            this->DeclareAbstractInputPort("query_object", Value<drake::geometry::QueryObject<double>>())
                                .get_index();

                        // Declare measurement output port.
                      /*  this->DeclareVectorOutputPort("depth2", kVectorValued,
                                                      &QuerySensor::CalcOutput);
                    }*/


                          //call event
                        this->DeclarePerStepPublishEvent(&QuerySensor::MyPublish);




                 /*   const InputPort<double>& QuerySensor::get_depth_input_port() const {
                      return this->get_input_port(0);
                    }*/



                   /* void QuerySensor::CalcOutput(const systems::Context<double> &context,
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

                    }*/

                }

                    EventStatus QuerySensor::MyPublish(
                        const Context<double> &context) const {
                      /*const auto params = dynamic_cast<const QuerySensor *>(
                        &context.get_numeric_parameter(0));
                        DRAKE_DEMAND(params != nullptr);*/
                        const auto &depth = query_sensor_input_port().Eval<drake::geometry::QueryObject<double>>(context);
                        auto val = depth.inspector().GetAllGeometryIds();
                        auto d = depth.GetPoseInWorld(val[4]).GetAsMatrix34();
                        auto d1 = d(1,3);
                        d = depth.GetPoseInWorld(val[2]).GetAsMatrix34();
                        auto d2 = d(1,3);
                        auto distance = d2 - d1;
                        if (distance < 0.5001)
                        {
                          drake::log()->info("Pick Object :)");
                        }
                        else{
                          drake::log()->info("Too far away :(");
                        }

                        //drake::log()->info(d);

                        //drake::log()->info("hello");

                        return EventStatus::Succeeded();
                    }


                    // namespace sensors
            }  // namespace systems
        }  // namespace drake
    }

}