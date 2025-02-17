from __future__ import annotations

import math
import typing

import phoenix6
import phoenix6.unmanaged
import wpilib
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import DCMotorSim
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import kilogram_square_meters

from components.drivetrain import SwerveModule

from generated.tuner_constants import TunerConstants

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class Falcon500MotorSim:
    def __init__(
        self,
        *motors: phoenix6.hardware.TalonFX,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be
        # greater than one.
        gearing: float,
        moi: kilogram_square_meters,
    ):
        self.falcon = DCMotor.falcon500(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(self.falcon, moi, gearing)
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.falcon)

    def update(self, dt: float) -> None:
        voltage = self.sim_states[0].motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.motor_sim.getAngularPosition() * motor_rev_per_mechanism_rad
            )
            sim_state.set_rotor_velocity(
                self.motor_sim.getAngularVelocity() * motor_rev_per_mechanism_rad
            )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: tuple[
            SwerveModule, SwerveModule, SwerveModule, SwerveModule
        ] = robot.chassis.modules

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive,
                units_per_rev=1 / 0.0503,
                kV=2.7,
            )
            for module in robot.chassis.modules
        ]
        self.steer = [
            Falcon500MotorSim(
                module.steer,
                gearing=1 / TunerConstants._steer_gear_ratio,
                # measured from MKCad CAD
                moi=0.0009972,
            )
            for module in robot.chassis.modules
        ]

        # Replace NavX with Pigeon 2
        self.current_yaw = 0.0
        self.gyro = robot.gyro.pigeon.sim_state  # Access the Pigeon 2's sim state
        self.gyro.set_supply_voltage(12.0)  # Set the supply voltage for simulation

        # self.imu = SimDeviceSim("navX-Sensor", 4)
        # self.imu_yaw = self.imu.getDouble("Yaw")

        # self.vision = VisionSystemSim("main")
        # self.vision.addAprilTags(game.apriltag_layout)
        # properties = SimCameraProperties.OV9281_1280_720()
        # self.camera = PhotonCameraSim(robot.vision.camera, properties)
        # self.camera.setMaxSightRange(5.0)
        # self.vision.addCamera(self.camera, robot.vision.robot_to_camera)

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Enable the Phoenix6 simulated devices
        # TODO: delete when phoenix6 integrates with wpilib
        if wpilib.DriverStation.isEnabled():
            phoenix6.unmanaged.feed_enable(0.1)

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds((
            self.swerve_modules[0].get(),
            self.swerve_modules[1].get(),
            self.swerve_modules[2].get(),
            self.swerve_modules[3].get(),
        ))

        self.current_yaw += math.degrees(speeds.omega * tm_diff)
        self.gyro.set_raw_yaw(self.current_yaw)

        self.physics_controller.drive(speeds, tm_diff)

        # self.vision.update(self.physics_controller.get_pose())
