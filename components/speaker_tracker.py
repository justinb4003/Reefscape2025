import wpilib
from photonlibpy import PhotonCamera

ps = wpilib.SmartDashboard.putString


class SpeakerTracker:
    def __init__(self):
        self.camera = PhotonCamera("orangepi02")
        self.speaker_detected = False
        self.desired_x = None
        self.desired_y = None
        self.desired_heading = None

    def execute(self):
        return
        res = self.camera.getLatestResult()
        # curr_heading = gyro.getHeading()
        curr_heading = 0
        self.note_detected = False
        if res.hasTargets():
            for idx, t in enumerate(res.getTargets()):
                class_id = t.objDetectId
                conf = t.objDetectConf
                bbox = t.detectedCorners
                ps(f"/photon/objs/{idx}", f"{class_id}, {conf}, {bbox}")
                if conf > 0.5:
                    offset = 10.0  # TODO: Calcualte something
                    self.desired_heading = curr_heading + offset
                    self.desired_x = (
                        2.0  # TODO: calculate based off bbox size and PID loop
                    )
                    self.desired_y = 1.0  # TODO: Calculate based off heading
                    self.note_detected = True
                    break

    def get_desired_xyz(
        self,
    ) -> tuple[float | None, float | None, float | None]:
        return self.desired_x, self.desired_y, self.desired_heading
