#include "drake/examples/multibody/SimpleArm/ObjectSensor.h"

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
            ObjectSensor::ObjectSensor(): LeafSystem<double>() {
                        // Declare depth input port.
                        object_sensor_input_port_ =
                            this->DeclareAbstractInputPort("query_object", Value<drake::geometry::QueryObject<double>>())
                                .get_index();

                        // Declare measurement output port.
                        //this->DeclareVectorOutputPort("depth2", kVectorValued, &ObjectSensor::CalcOutput);

                        this-> DeclareVectorOutputPort("distance", BasicVector<double>(1),
                                                 &ObjectSensor::CalcOutput);
                    }


                          //call event
                       // this->DeclarePerStepPublishEvent(&ObjectSensor::MyPublish);




                 /*   const InputPort<double>& ObjectSensor::get_depth_input_port() const {
                      return this->get_input_port(0);
                    }*/



                   /* void ObjectSensor::CalcOutput(const systems::Context<double> &context,
                                                    BasicVector<double> *output) const {
                        const auto params = dynamic_cast<const ObjectSensor *>(
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

                   /* EventStatus ObjectSensor::MyPublish(
                        const Context<double> &context) const {
                      *//*const auto params = dynamic_cast<const ObjectSensor *>(
                        &context.get_numeric_parameter(0));
                        DRAKE_DEMAND(params != nullptr);*//*
                        const auto &depth = query_sensor_input_port().Eval<drake::geometry::QueryObject<double>>(context);
                        auto val = depth.inspector().GetAllGeometryIds();
                        auto d = depth.GetPoseInWorld(val[4]).GetAsMatrix34();
                        auto d1 = d(1,3);
                        d = depth.GetPoseInWorld(val[2]).GetAsMatrix34();
                        auto d2 = d(1,3);
                        auto distance = d2 - d1;

                       *//* if (distance < 0.5001)
                        {
                          drake::log()->info("Pick Object :)");
                        }
                        else{
                          drake::log()->info("Too far away :(");
                        }*//*

                        //drake::log()->info(d);

                        //drake::log()->info("hello");

                        return EventStatus::Succeeded();
                    }*/

                    //void ObjectSensor::CalcOutput(const Context<double> &context, Eigen::VectorBlock<VectorX<double>>* output) const
                      void SimpleArm::ObjectSensor::CalcOutput(const Context<double>& context, BasicVector<double>* output) const{

                      const auto &depth = object_sensor_input_port().Eval<drake::geometry::QueryObject<double>>(context);
                      auto val = depth.inspector().GetAllGeometryIds();
                      auto d_gripper = depth.GetPoseInWorld(val[8]).GetAsMatrix34();
                      auto d1x = d_gripper(1,3);
                      auto d1z = d_gripper(2,3);
                      auto d_brick1 = depth.GetPoseInWorld(val[10]).GetAsMatrix34();
                      auto d2x = d_brick1(1,3);
                      //auto d_brick2 = depth.GetPoseInWorld(val[7]).GetAsMatrix34();
                      //drake::log()->info(d1z);
                      double SensorHeight = 100;
                      double dist = NAN;
                      if (0 <= d1z && d1z > SensorHeight && (d1x >= 0)) {
                        dist = fabs(d2x - d1x);
                      }
                      else {
                        dist = fabs(d2x);
                      }

                      //drake::log()->info(dist);
                      //auto dist = d2x;
                      BasicVector<double> d3(1);
                      d3[0] = dist;
                      //drake::log()->info(context.get_time() - t0);
                      output->get_mutable_value() = d3.value();
                    }


                    // namespace sensors
            }  // namespace systems
        }  // namespace drake
    }

