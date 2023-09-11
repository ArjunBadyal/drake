#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/query_object.h"


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
                    class ObjectSensor final : public LeafSystem<double> {
                    public:
                        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ObjectSensor)

                        ObjectSensor();

                        const systems::InputPort<double>& object_sensor_input_port() const {
                          return this->get_input_port(object_sensor_input_port_);
                        }

                    private:
                        /*void CalcOutput(const Context<double>& context, BasicVector<double>* output) const;
                        */
                     int object_sensor_input_port_;
                     //event handler
                     //EventStatus MyPublish(const Context<double>&) const;

                     void CalcOutput(const Context<double>& context,
                                          BasicVector<double>* distance) const;

                    };


                }  // namespace sensors

            }  // namespace systems
        }  // namespace drake
    }
