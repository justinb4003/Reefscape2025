import enum


@enum.unique
class TalonId(enum.IntEnum):
    """CAN ID for CTRE Talon motor controllers (e.g. Talon FX, Talon SRX)."""

    DRIVE_FR = 11
    DRIVE_FL = 12
    DRIVE_BL = 13
    DRIVE_BR = 14

    TURN_FR = 21
    TURN_FL = 22
    TURN_BL = 23
    TURN_BR = 24

    AMP_LIFT_MOTOR = 54

    SHOOTER_MOTOR_LEFT = 41
    SHOOTER_MOTOR_RIGHT = 42


@enum.unique
class CancoderId(enum.IntEnum):
    """CAN ID for CTRE CANcoder."""

    SWERVE_FR = 31
    SWERVE_FL = 32
    SWERVE_BL = 33
    SWERVE_BR = 34


@enum.unique
class CanId(enum.IntEnum):
    """CAN IDs for miscellaneous devices."""

    PIGEON = 63


@enum.unique
class SparkId(enum.IntEnum):
    """CAN ID for REV SPARK motor controllers (Spark Max, Spark Flex)."""

    INTAKE_MOTOR_FEED = 51
    AMP_FEED_MOTOR = 55

    CLIMBER_MOTOR_LEFT = 57
    CLIMBER_MOTOR_RIGHT = 58

    SHOOTER_MOTOR_FEED_RIGHT = 46
    SHOOTER_MOTOR_FEED_LEFT = 47

    SHOOTER_MOTOR_TILT_LEFT = 44
    SHOOTER_MOTOR_TILT_RIGHT = 45


@enum.unique
class DioChannel(enum.IntEnum):
    """roboRIO Digital I/O channel number."""


@enum.unique
class PwmChannel(enum.IntEnum):
    """roboRIO PWM output channel number."""
