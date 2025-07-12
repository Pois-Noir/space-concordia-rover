
from pydrake.all import DiagramBuilder
from pydrake.all import AddMultibodyPlantSceneGraph
from pydrake.all import MultibodyPlant
from pydrake.all import RotationMatrix
from pydrake.all import Parser
from pydrake.all import Simulator
from pydrake.all import StartMeshcat
from pydrake.all import AddDefaultVisualization
from pydrake.all import Solve
from pydrake.all import RigidTransform
from pydrake.all import RandomGenerator
import numpy as np

from pydrake.multibody.inverse_kinematics import InverseKinematics


#   -----------------------------------------------------
#   Loading robot and building the world

meshcat = StartMeshcat()


sim_time_step = 0.01

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(
    builder, time_step=sim_time_step
)
parser = Parser(plant)

urdf_path = "../ceres.xacro"

f = open(urdf_path, "r")
robot_urdf_string = f.read()
robot_urdf_string.replace("\n", "")
Parser(plant).AddModelsFromString(robot_urdf_string, "urdf")

robot_base = plant.GetFrameByName("base_structure_link")
plant.WeldFrames(
    plant.world_frame(),
    robot_base,
    RigidTransform.Identity()
)

plant.Finalize()

AddDefaultVisualization(builder=builder, meshcat=meshcat)

diagram = builder.Build()
#   -------------------------------------------------------------
#   Creating Simulation

simulator = Simulator(diagram)
simulator_context = simulator.get_context()

plant_context = plant.GetMyMutableContextFromRoot(simulator_context)

#   ------------------------------------------------------------

plant.SetPositions(plant_context, np.array(
    [0,   0,      0,     0,     0]
))

# Step 4: Print the result


#   ------------------------------------------------------------
#   Simulation Section

simulator.Initialize()
simulator.set_target_realtime_rate(1.0)

meshcat.StartRecording()

while True:
    # Adjust joint values dynamically if needed

    # Advance the simulation in small time steps
    simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)

#   ------------------------------------------------------------
#   move plant
