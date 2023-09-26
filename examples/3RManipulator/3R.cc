#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include <iostream>

#include <chrono>
#include <thread>


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
#include "drake/multibody/plant/multibody_plant.h"

#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include <iostream>





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




DEFINE_double(target_realtime_rate, 0.3,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time,5,
              "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0.0001,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");


int do_main() {

    //add scene graph system to builder
  systems::DiagramBuilder<double> builder;



            MultibodyPlant<double>& SimpleArm =
                    *builder.AddSystem<MultibodyPlant>(FLAGS_time_step); //add multibody plant system to builder

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>(); //add scene graph to builder
  scene_graph.set_name("scene_graph");





  //TODO PMHSystem.control_channels = (control_Channels*)malloc(sizeof(control_Channels));
  //


  // Make and add the SimpleArm model.
  const std::string body1 = FindResourceOrThrow(
      "drake/examples/3RManipulator/3R.sdf");



  Parser(&SimpleArm, &scene_graph).AddModelFromFile(body1); //adds model to the scene graph and the multibody plant from SDF file


    //Assign joint model instances to variables for ease of access
    const RevoluteJoint<double>& Joint1 =
            SimpleArm.GetJointByName<RevoluteJoint>("joint1");
    /*const RevoluteJoint<double>& WristJoint =
            SimpleArm.GetJointByName<RevoluteJoint>("6-WristJoint");*/

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
   const auto& root_link = SimpleArm.GetBodyByName("base_link");
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

//drake::log()->info(SimpleArm.get_actuation_input_port().get_name());

            /*auto Vel = builder.AddSystem<VelCmd<double>>();
            Vel->set_name("input trajectory");
//builder.Connect(SimpleArm.get_output_port(0), InDyn->get_input_port_estimated_state());
            builder.Connect(Vel->get_output_port(),SimpleArm.get_input_port(3)); //Works! ^_^*/

  //builder.Connect(PMHSystem -> get_output_port(), SimpleArm.get_input_port(3));

  //Now connect input port from the multibody plant to output port of the scene graph
  builder.Connect(
      SimpleArm.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(SimpleArm.get_source_id().value()));

  //geometry
    builder.Connect(scene_graph.get_query_output_port(), SimpleArm.get_geometry_query_input_port());

    // adds visualizer to builder and connects it the output of the scene graph
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);





  auto diagram = builder.Build();


  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& SimpleArm_context = diagram->GetMutableSubsystemContext(SimpleArm, diagram_context.get());
  //Run this to get info about the mbp port:
  /*drake::log()->info("name");
  drake::log()->info(SimpleArm.get_input_port(3).get_name());*/


    // Set initial state.
    Joint1.set_angle(&SimpleArm_context, 0.0);
    //WristJoint.set_angle(&SimpleArm_context, 0.0);
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  //lcm::Subscribe();
  simulator.set_publish_every_time_step(true) ;
  simulator.set_publish_at_initialization(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(8);
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
  while (LoopCond) {
    //auto t = simulator.get_context().get_time();
    //drake::log()->info(t);
    //auto x = SimpleArm.GetBodyByName("1-BaseLink").body_frame().CalcPose(SimpleArm_context, SimpleArm.world_frame());
    //drake::log()->info(x);

    //LoopCond = false;
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
  using std::chrono::system_clock;

  std::this_thread::sleep_until(system_clock::now() + std::chrono::seconds(3));

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