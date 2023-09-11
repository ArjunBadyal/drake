#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
    using systems::LeafSystem;
    using systems::InputPort;
    using systems::BasicVector;
    using systems::Context;
    namespace examples {
        namespace multibody {
            namespace SimpleArm {



                    class QuerySensorNoOutputPort final : public LeafSystem<double> {
                    public:
                        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuerySensor)

                        QuerySensorNoOutputPort();




                        const systems::InputPort<double>& query_sensor_input_port() const {
                          return this->get_input_port(query_sensor_input_port_);
                        }

                    private:
                        void CalcOutput(const Context<double>& context, BasicVector<double>* output) const;
                        int query_sensor_input_port_{};
                    };



                }  // namespace sensors

            }  // namespace systems
        }  // namespace drake
    }
