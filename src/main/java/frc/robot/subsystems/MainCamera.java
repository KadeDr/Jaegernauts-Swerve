package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class MainCamera {
    private PhotonCamera camera;

    @SuppressWarnings("unused")
    private double lowerPosition = 1;

    private XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);

    private static final double CAMERA_HEIGHT_METERS = 0.6;
    private static final double TARGET_HEIGHT_METERS = 1.5;
    private static final double CAMERA_PITCH_RADIANS = Math.toRadians(20);
    private static final double FIELD_TAG_POSITION_X = 3.0;
    private static final double FIELD_TAG_POSITION_Y = 2.0;

    AprilTagFieldLayout aprilTagFieldLayout;
    ArrayList<AprilTag> ApriList;
    double fieldlength = 651.25;
    double fieldwidth = 315.5;

    private final double VISION_TURN_kP = 0.01;
    private final double VISION_DES_ANGLE_deg = 0.0;
    private final double VISION_STRAFE_kP = 0.5;
    private final double VISION_DES_RANGE_m = 1.25;

    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    int targetId = 0;
    double yaw = 0, pitch = 0, area = 0, skew = 0, range, robotX, robotY;
    Transform3d pose;
    List<TargetCorner> corners;
    Pose3d robotPose;

    public void InitializeCamera() {
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    }

    public void CameraRobotPeriodic() {
        result = camera.getLatestResult();

        if (result.hasTargets()) {
            target = result.getBestTarget();

            targetId = target.getFiducialId();
            yaw = target.getYaw();
            pitch = target.getPitch();
            area = target.getArea();
            skew = target.getSkew();
            pose = target.getBestCameraToTarget();
            corners = target.getDetectedCorners();

            range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS, Math.toRadians(pitch));

            robotX = FIELD_TAG_POSITION_X - range * Math.cos(Math.toRadians(yaw));
            robotY = FIELD_TAG_POSITION_Y - range * Math.sin(Math.toRadians(yaw));

            SmartDashboard.putNumber("TargetId", targetId);
            SmartDashboard.putNumber("Yaw", yaw);
            SmartDashboard.putNumber("Pitch", pitch);
            SmartDashboard.putNumber("Range", range);
            SmartDashboard.putNumber("Robot X", robotX);
            SmartDashboard.putNumber("Robot Y", robotY);

            // Current test for estimating field relative pose with april tags
            if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), pose);
            }
        } else {
            SmartDashboard.putString("Status", "No targets detected");
        }
    }

    public void CameraTeleopPeriodic(double strafe) {
        double forward = 0, turn = 0;
        boolean aButton = m_driveController.getAButton();
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 2) {
                        targetYaw = target.getYaw();
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(0.5, 0.435,
                                Units.degreesToRadians(-30), Units.degreesToRadians(target.getPitch()));
                        targetVisible = true;
                    }
                }
            }
        }

        if (targetVisible) {
            turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.DriveConstants.kMaxAngularSpeed;
            forward = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP
                    * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        }

        SmartDashboard.putNumber("Forward", forward);
        SmartDashboard.putNumber("Strafe", strafe);
        SmartDashboard.putNumber("Turn", turn);

    }
}