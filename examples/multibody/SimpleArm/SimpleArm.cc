#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include <iostream>


#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/single_output_vector_source.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
//#include "drake/systems/primitives/constant_vector_source.h"
//#include "drake/systems/primitives/trajectory_source.h"

//#include "drake/examples/multibody/SimpleArm/SimpleLCM.h"
#include "drake/examples/multibody/SimpleArm/ObjectSensor.h"
#include "PlatformMappingHandler.h"

//#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include <iostream>
#include "drake/geometry/query_object.h"





namespace drake {
    using Eigen::Vector2d;

  //  using Message = lcmt_drake_signal;
    using multibody::BodyIndex;
    using multibody::ForceElementIndex;
    using multibody::Joint;
    using multibody::MultibodyPlant;
    using multibody::Parser;
    using multibody::RevoluteJoint;
    using systems::BasicVector;
    using systems::Context;
    using systems::DiagramBuilder;
    using systems::LeafSystem;
    using systems::InputPort;
    using systems::OutputPort;
    using systems::OutputPortIndex;
    using systems::Simulator;
namespace examples {
namespace multibody {
namespace SimpleArm {


using geometry::SceneGraph;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;


std::string input;
double t0 = 0;
double t1 = 0;
bool LoopCond = true;




DEFINE_double(target_realtime_rate, 0.1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time,5,
              "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0.0001,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

/*DEFINE_double(penetration_allowance, 1.0e-2,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");*/



        /*class VelCmd: public LeafSystem<double>{

        public:

        VelCmd(const MultibodyPlant<double>* plant);

            const InputPort<double>& get_input_port_estimated_state() const {
                return this->get_input_port(input_port_index_state_);
            }

            //Calculates the output velocity
            const OutputPort<double>& get_output_port_velocity() const {
                return this->get_output_port(output_port_index_velocity_);
            }

        private:
            void CalcOutputVelocity(const Context<double>& context,
                                    BasicVector<double>* velocity) const;

            // Methods for updating cache entries.
            void SetMultibodyContext(const Context<double>&, Context<double>*) const;


            //Multibody plant declaration
            const MultibodyPlant<double>* const multibody_plant_;

            // This is the calculator method for the output port.
            int input_port_index_state_{0};

            int output_port_index_velocity_{0};
            const int v_dim_{0};
            const int q_dim_{0};
            drake::systems::CacheIndex multibody_plant_context_cache_index_;

        };

        VelCmd::VelCmd(const MultibodyPlant<double>* plant): multibody_plant_(plant),
                                                             v_dim_(plant->num_velocities()) {

            DRAKE_DEMAND(plant->is_finalized());
            input_port_index_state_ =
                    this->DeclareInputPort(systems::kUseDefaultName, systems::kVectorValued, q_dim_ + v_dim_)
                            .get_index();
            output_port_index_velocity_ =
                    this->DeclareVectorOutputPort("OutputVel", 2,
                                                  &VelCmd::CalcOutputVelocity)
                            .get_index();

            auto multibody_plant_context = multibody_plant_->CreateDefaultContext();
            multibody_plant_context_cache_index_ =
                    this->DeclareCacheEntry(
                                    "multibody_plant_context_cache",
                                    *multibody_plant_context,
                                    &VelCmd::SetMultibodyContext,
                                    {this->input_port_ticket(
                                            get_input_port_estimated_state().get_index())})
                            .cache_index();

        }

        void VelCmd::SetMultibodyContext(
                const Context<double>& context,
                Context<double>* multibody_plant_context) const {
            const VectorX<double>& x = get_input_port_estimated_state().Eval(context);
                // Set the plant positions and velocities.
                multibody_plant_->SetPositionsAndVelocities(multibody_plant_context, x);

        }

        void VelCmd::CalcOutputVelocity(const Context<double>& context, BasicVector<double>* output) const {
            const auto& multibody_plant_context =
                    this->get_cache_entry(multibody_plant_context_cache_index_).Eval<Context<double>>(context);


            Eigen::Vector2d other;
            other << 10000*Operation.PrePlace(context.get_time()),multibody_plant_context.get_time() * 1000 ;
            drake::log()->info(other);
            output ->get_mutable_value() = other;

        }*/
/*
  */
/*
 class SingleOutputVectorSource : public LeafSystem<double>
    {

    };



    template<typename T>
    class VelCmd final : public systems::SingleOutputVectorSource<T> {
    public:
        VelCmd();

        ~VelCmd() final = default;

    private:
        void DoCalcVectorOutput(
                const Context<T> &context,
                Eigen::VectorBlock<VectorX<T>>* output) const final;

    };


    template<typename T>
    VelCmd<T>::VelCmd():systems::SingleOutputVectorSource<T>(2) {

    }

    template<typename T>
    void
    VelCmd<T>::DoCalcVectorOutput(const Context<T> &context, Eigen::VectorBlock<VectorX<T>>* output) const  {

        Eigen::Vector2d command;
        command << Operation(context.get_time() - t0), 0;
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

    template class VelCmd<double>;

*/



int do_main() {

    //add scene graph system to builder
  systems::DiagramBuilder<double> builder;



            MultibodyPlant<double>& SimpleArm =
                    *builder.AddSystem<MultibodyPlant>(FLAGS_time_step); //add multibody plant system to builder

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>(); //add scene graph to builder
  scene_graph.set_name("scene_graph");




  auto ObjectSensorSystem = builder.AddSystem<ObjectSensor>();

  auto PMHSystem = builder.AddSystem<PMH>();


  // Make and add the SimpleArm model.
  const std::string body1 = FindResourceOrThrow(
      "drake/examples/multibody/SimpleArm/SimpleArm.sdf");

  const std::string body2 = FindResourceOrThrow(
          "drake/examples/multibody/SimpleArm/brick.sdf");
  const std::string body3 = FindResourceOrThrow(
      "drake/examples/multibody/SimpleArm/brick2.sdf");

  Parser(&SimpleArm, &scene_graph).AddModelFromFile(body1); //adds model to the scene graph and the multibody plant from SDF file
  Parser(&SimpleArm, &scene_graph).AddModelFromFile(body2);
  Parser(&SimpleArm, &scene_graph).AddModelFromFile(body3);

    //Assign joint model instances to variables for ease of access
    const RevoluteJoint<double>& ElbowJoint =
            SimpleArm.GetJointByName<RevoluteJoint>("7-ElbowJoint");
    const RevoluteJoint<double>& WristJoint =
            SimpleArm.GetJointByName<RevoluteJoint>("6-WristJoint");

    // Add model of the ground.
    const double static_friction = 1.0;
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
   SimpleArm.RegisterVisualGeometry(SimpleArm.world_body(), math::RigidTransformd(),
                                 geometry::HalfSpace(), "GroundVisualGeometry",
                                 green);

    // For a time-stepping model only static friction is used.
    const drake::multibody::CoulombFriction<double> ground_friction(static_friction,
                                                             static_friction);
    /*SimpleArm.RegisterCollisionGeometry(SimpleArm.world_body(), math::RigidTransformd(),
                                    geometry::HalfSpace(),
                                    "GroundCollisionGeometry", ground_friction);*/

   //Weld base to world frame
   const auto& root_link = SimpleArm.GetBodyByName("1-BaseLink");
            SimpleArm.AddJoint<drake::multibody::WeldJoint>("weld_base", SimpleArm.world_body(), std::nullopt,
                                    root_link, std::nullopt,
                                math::RigidTransform<double>::Identity());

            const Vector3<double> g(0,0,0);

           SimpleArm.mutable_gravity_field().set_gravity_vector(g);
  // Now the model is complete.
  SimpleArm.Finalize();
 // SimpleArm.set_penetration_allowance(FLAGS_penetration_allowance);
  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(SimpleArm.geometry_source_is_registered());

//drake::log()->info(SimpleArm.get_actuation_input_port().size());
            /*auto Vel = builder.AddSystem<VelCmd<double>>();
            Vel->set_name("input trajectory");
//builder.Connect(SimpleArm.get_output_port(0), InDyn->get_input_port_estimated_state());
            builder.Connect(Vel->get_output_port(),SimpleArm.get_input_port(3)); //Works! ^_^*/

  builder.Connect(PMHSystem -> get_output_port(), SimpleArm.get_input_port(3));

  //Now connect input port from the multibody plant to output port of the scene graph
  builder.Connect(
      SimpleArm.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(SimpleArm.get_source_id().value()));

  //geometry
    builder.Connect(scene_graph.get_query_output_port(), SimpleArm.get_geometry_query_input_port());

    // adds visualizer to builder and connects it the output of the scene graph
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  //Connect Scene graph to ObjectSensorSystem
  builder.Connect(scene_graph.get_query_output_port(), ObjectSensorSystem->object_sensor_input_port());

  builder.Connect(ObjectSensorSystem ->get_output_port(), PMHSystem->get_input_port());




  auto diagram = builder.Build();


  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& SimpleArm_context = diagram->GetMutableSubsystemContext(SimpleArm, diagram_context.get());



    // Set initial state.
    ElbowJoint.set_angle(&SimpleArm_context, 0.0);
    WristJoint.set_angle(&SimpleArm_context, 0.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  //lcm::Subscribe();
  simulator.set_publish_every_time_step(true) ;
  simulator.set_publish_at_initialization(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

   /* while(LoopCond) {

        std::cout << "Enter Operation:" << std::endl;
        std::cin >> input;
        //input = std::cin.get();
        if (input == "PrePick") {
            std::cout << "PrePicking" << std::endl;
            t1 = t0 + 2;
        }
        else if(input == "PrePlace"){
            std::cout << "PrePlacing" << std::endl;
            t1 = t0 + 2;
            //TODO Remove object from Pick side and add goal to place side
        }
        else if(input == "Return"){
            std::cout << "Returning" << std::endl;
            t1 = t0 + 3;
            //TODO Add object to Place side
        }
        else {
            std::cout << "Invalid Input" << std::endl;
        }
        simulator.AdvanceTo(t1);
        t0 = simulator.get_context().get_time();
        drake::log()->info(t0);
    }
            LoopCond = false;
*/
  while (true) {
    auto t = simulator.get_context().get_time();
    drake::log()->info(t);
    simulator.AdvanceTo(t + 0.001);
  }
 //const Context<double>& SimContext = simulator.get_context();
  return 0;
}

 // namespace
}  // namespace SimpleArm
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple robotic arm demo using Drake's MultibodyPlant,"
      "with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::SimpleArm::do_main();
}





//Alternative method for message passing using the default handler:
/*  int dimension;
  drake::lcm::Subscribe<Message>(&lcm_,"channel",[&](const Message& message) {
      DRAKE_THROW_UNLESS(message.dim > 0);
      dimension = message.dim;}, [](){drake::log()->info("can't log");}
      );
  drake::lcm::Publish(&lcm_,"channel" , sample_);*/


//Message passing test

/* drake::log()->info(Operation.PrePick(0.5));
 drake::lcm::DrakeLcm lcm_;
 lcmt_drake_signal sample_{2,{9,2},{"a","b"}};



   lcm::SubMailbox("channel", &lcm_);
   drake::lcm::Publish(&lcm_,"channel" , sample_);

   //lcm_.HandleSubscriptions(0);
   lcm_.HandleSubscriptions(10);*/