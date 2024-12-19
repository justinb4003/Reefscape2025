import math

import magicbot
import wpilib
import wpilib.event
from magicbot import tunable
from wpimath.geometry import Rotation3d, Translation3d

from components.drivetrain import DrivetrainComponent
from components.note_tracker import NoteTracker
from components.speaker_tracker import SpeakerTracker
from components.vision import VisualLocalizer
from utilities.game import is_red
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers

    # Components
    chassis: DrivetrainComponent
    vision: VisualLocalizer
    note_tracker: NoteTracker
    speaker_tracker: SpeakerTracker

    max_speed = magicbot.tunable(5)  # m/s
    lower_max_speed = magicbot.tunable(2)  # m/s
    max_spin_rate = magicbot.tunable(4)  # m/s
    lower_max_spin_rate = magicbot.tunable(2)  # m/s
    inclination_angle = tunable(0.0)

    START_POS_TOLERANCE = 1

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        # side: (28*3)*2 + front: (30*3) - 2 (R.I.P)
        self.status_lights_strip_length = (28 * 3) * 2 + (30 * 3) - 2

        self.vision_name = "ardu_cam"
        self.vision_pos = Translation3d(0.25, 0.0, 0.20)
        self.vision_rot = Rotation3d(0, -math.radians(20), 0)

    def teleopInit(self) -> None:
        self.field.getObject("Intended start pos").setPoses([])

    def handle_drivetrain(self) -> None:
        # foreshadowing function in this commit!
        pass

    def teleopPeriodic(self) -> None:
        # Set max speed
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate
        if self.gamepad.getRightBumper():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate

        if self.gamepad.getAButton() and self.note_tracker.note_detected:
            # Note Tracking
            x, y, z = self.note_tracker.get_desired_xyz()
            self.chassis.drive_local(x, y, z)
        elif self.gamepad.getBButton() and self.speaker_tracker.speaker_detected:
            # Speaker Tracking
            x, y, z = self.speaker_tracker.get_desired_xyz()
            self.chassis.drive_local(x, y, z)
        else:
            # Driving
            drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 2.5) * max_speed
            drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 2.5) * max_speed
            drive_z = (
                -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2)
                * max_spin_rate
            )
            local_driving = self.gamepad.getXButton()

            if local_driving:
                self.chassis.drive_local(drive_x, drive_y, drive_z)
            else:
                if is_red():
                    drive_x = -drive_x
                    drive_y = -drive_y
                self.chassis.drive_field(drive_x, drive_y, drive_z)
            # Give rotational access to the driver
            if drive_z != 0:
                self.chassis.stop_snapping()

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        dpad = self.gamepad.getPOV()
        if dpad != -1:
            if is_red():
                self.chassis.snap_to_heading(-math.radians(dpad) + math.pi)
            else:
                self.chassis.snap_to_heading(-math.radians(dpad))
        else:
            self.chassis.stop_snapping()
            self.chassis.drive_local(0, 0, 0)

        self.chassis.execute()

        self.chassis.update_odometry()

        self.vision.execute()

    def disabledPeriodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

        self.vision.execute()
