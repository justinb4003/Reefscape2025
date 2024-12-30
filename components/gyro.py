from wpimath.geometry import Rotation2d
from phoenix6.hardware import Pigeon2
from ids import CanId


class Gyro:
    def __init__(self):
        self.pigeon = Pigeon2(CanId.PIGEON)

    def get_heading(self) -> float:
        return self.pigeon.get_yaw().value

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    def execute(self) -> None:
        pass