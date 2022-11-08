#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
    using systems::LeafSystem;
    using systems::InputPort;
    using systems::OutputPort;
    using systems::BasicVector;
    using systems::Context;
    namespace examples {
        namespace multibody {
            namespace SimpleArm {

                    template<typename T>
                    class QueryObject final : public LeafSystem<T> {
                    public:
                        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QueryObject)

                        QueryObject();

                        template <typename U>
                        explicit QueryObject(const QueryObject<U>&);


                        const InputPort<T> &get_depth_input_port() const {
                            return this->get_input_port(0);
                        }


                    private:
                        void CalcOutput(const Context<T> &context, BasicVector<T> *output) const;

                    };



                }  // namespace sensors

            }  // namespace systems
        }  // namespace drake
    }
