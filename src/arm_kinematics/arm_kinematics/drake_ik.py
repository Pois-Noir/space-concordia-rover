import numpy as np
from pydrake.all import MultibodyPlant, RotationMatrix, Parser, Solve, RigidTransform
from pydrake.multibody.inverse_kinematics import InverseKinematics
import math

def rotateAroundZ(transform: np.ndarray, rotationRadians: float) -> np.ndarray:
    rotationOnlyTransform = np.copy(transform)
    rotationOnlyTransform[:3, 3] = 0.0

    appliedRotationTransform = np.array([
        [math.cos(rotationRadians), -math.sin(rotationRadians), 0.0, 0.0],
        [math.sin(rotationRadians), math.cos(rotationRadians), 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    assert appliedRotationTransform is not None

    newTransform = np.matmul(rotationOnlyTransform, np.linalg.inv(appliedRotationTransform))
    newTransform[:3, 3] = transform[:3, 3]

    return newTransform


# https://drake.mit.edu/
class KinematicsEngineDrake:

    def __init__(self, urdfPath, baseJointName, lastJointName, timeStep):
        drakePlant = MultibodyPlant(timeStep)

        robotUrdfAsString = open(urdfPath, "r").read()
        robotUrdfAsString.replace("\n", "")

        Parser(drakePlant).AddModelsFromString(robotUrdfAsString, "urdf")

        # To prevent kinematic solutions where the robot floats in space,
        # drake requires pinning down the robot base to a fixed position.
        robot_base = drakePlant.GetFrameByName(baseJointName)
        drakePlant.WeldFrames(
            drakePlant.world_frame(),
            robot_base,
            RigidTransform.Identity()
        )

        drakePlant.Finalize()

        drakePlantContext = drakePlant.CreateDefaultContext()

        self.plant = drakePlant
        self.context = drakePlantContext
        self.lastJoint = lastJointName

    def forwardKinematics(self, jointsInDegrees: np.ndarray):

        self.plant.SetPositions(self.context, np.radians(jointsInDegrees))

        endEffectorTransform = self.plant.CalcRelativeTransform(
            self.context,
            self.plant.world_frame(),
            self.plant.GetFrameByName(self.lastJoint)
        ).GetAsMatrix4()

        endEffectorTransformMillimeter = endEffectorTransform

        return endEffectorTransformMillimeter

    """
        Success rate : 94%
        Problems : 
            - The current implementation of Drake lacks the necessary precision, particularly when joint constraints 
            are applied for the injection.

            - Adding Tool in drake caused problems for forward kinematics due to the difference in the drake world and
            VetRobot world 
        TODOs :
            - Develop a method to constrain the algorithm for greater precision.

            - If adding tool in Drake is implemented, the fixed orientation constraint can be replaced with 
            AddAngleBetweenVectorsConstraint, potentially reducing errors.
    """
    def inverseKinematics(self,
                          flangeTransform,
                          positionError=1e-6,
                          orientationError=1e-6):

        solutionJointsDegrees, didInverseKinematicsConverge = self.inverseKinematicsAttempt(
            flangeTransform,
            positionError,
            orientationError,
        )

        # If inverse kinematics fails the first time, we try to solve a simpler problem, without the orientation constraints.
        # Then we use this intermediate solution as the starting point (guess) for a second attempt.
        # Tests show that this approach helps reduce the number of cases where inverse kinematics fails.
        if not didInverseKinematicsConverge:
            # Even though the inverse kinematics without orientation may fail,
            # it might still succeed when using the failed result as the initial guess.
            initialGuess, _ = self.inverseKinematicsAttempt(
                flangeTransform,
                positionError,
                orientationError,
                addOrientationConstraint=False
            )

            solutionJointsDegrees, didInverseKinematicsConverge = self.inverseKinematicsAttempt(
                flangeTransform,
                positionError,
                orientationError,
                initialGuess=initialGuess
            )

        return solutionJointsDegrees, didInverseKinematicsConverge

    def inverseKinematicsAttempt(self,
                                 flangeTransform: np.ndarray,
                                 positionError,
                                 orientationError,
                                 addOrientationConstraint=True,
                                 initialGuess=None
                                 ):

        destinationTranslation = np.array(flangeTransform[:3, 3])

        translationLowerBound = destinationTranslation - positionError
        translationUpperBound = destinationTranslation + positionError

        desired_orientation = RotationMatrix(flangeTransform[:3, :3])

        drakeInverseKinematics = InverseKinematics(self.plant, self.context)

        # We might attempt to solve the IK without the orientation constraint
        # to obtain an initial guess for the solution.
        if addOrientationConstraint:
            drakeInverseKinematics.AddOrientationConstraint(
                self.plant.world_frame(),
                desired_orientation,
                self.plant.GetFrameByName(self.lastJoint),
                RotationMatrix(np.eye(3)),
                orientationError
            )

        drakeInverseKinematics.AddPositionConstraint(
            self.plant.GetFrameByName(self.lastJoint),
            [0.0, 0.0, 0.0],
            self.plant.world_frame(),
            translationLowerBound,
            translationUpperBound
        )

        drakeInverseKinematicsProgram = drakeInverseKinematics.prog()
        drakeInverseKinematicsVariable = drakeInverseKinematics.q()

        # Include the joint guesses if we have them available.
        if initialGuess is not None:
            drakeInverseKinematicsProgram.SetInitialGuess(drakeInverseKinematicsVariable, np.radians(initialGuess))

        result = Solve(drakeInverseKinematicsProgram)

        didInverseKinematicsConverge = result.is_success()

        solutionJointsRadians = result.GetSolution(drakeInverseKinematicsVariable)
        solutionJointsDegrees = np.degrees(solutionJointsRadians)

        return solutionJointsDegrees, didInverseKinematicsConverge

def main():

    drake_kinematics = KinematicsEngineDrake(
        "./ceres.xacro",
        "base_structure_link",
        "gripper_claw_link",
        0.01
    )



    home = drake_kinematics.forwardKinematics([0,   0,  0,  0,  0])

    joints, is_done = drake_kinematics.inverseKinematicsAttempt(home,1e-6, 1e-6)

    print(home)

    new_home = drake_kinematics.forwardKinematics(joints)

    print(is_done)
    print(joints)

    print(new_home - home)

main()