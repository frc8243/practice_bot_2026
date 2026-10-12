package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;

public class Vision {

    private final String limelightName;
    private double lastTimestamp = -1;
    private int acceptCount = 0;
    private int rejectCount = 0;
    private final StructPublisher<Pose2d> posePublisher;

    public Vision(String limelightName) {
        this.limelightName = limelightName;
        // System.out.println("limelight " + limelightName);

        // set camera position on robot - measure these values!
        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                Units.inchesToMeters(6.0), // forward from robot center (meters, + = forward)
                Units.inchesToMeters(0.0), // left from robot center (meters, + = left)
                Units.inchesToMeters(14.75), // up from floor (meters)
                0.0, // roll (degrees)
                0.0, // pitch (degrees, + = tilted back)
                0.0); // yaw (degrees, + = rotated left)

        posePublisher =
                NetworkTableInstance.getDefault()
                        .getStructTopic("VisionPoseEstimator/" + limelightName, Pose2d.struct)
                        .publish();
        posePublisher.setDefault(new Pose2d());
    }

    public void updatePose(SwerveDrive drive) {
        // required for MegaTag2 to work
        LimelightHelpers.SetRobotOrientation(
                limelightName, drive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // reject if spinning too fast (> 2 rot/sec)
        if (Math.abs(drive.getRobotVelocity().omegaRadiansPerSecond)
                > Math.toRadians(360)) { // 2 * Math.PI * 2
            reject("spinning to fast");
            return;
        }

        double linearSpeed =
                Math.hypot(
                        drive.getRobotVelocity().vxMetersPerSecond,
                        drive.getRobotVelocity().vyMetersPerSecond);

        // reject if driving to0 fast, > 80% of robot speed!
        if (linearSpeed > 0.8 * drive.getMaximumChassisVelocity()) {
            reject("driving too fast");
            return;
        }

        // System.out.println("VISION " + limelightName);

        var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (est == null) {
            reject("null");
            return;
        }
        if (est.tagCount < 1) {
            reject("no tag");
            return;
        }
        SmartDashboard.putNumber("Vision/tagCount", est.tagCount);
        SmartDashboard.putNumber("Vision/avgTagDist", est.avgTagDist);
        // SmartDashboard.putNumber("Vision/latencySec",
        // Timer.getFPGATimestamp() - est.timestampSeconds);

        if (est.pose.getX() == 0 && est.pose.getY() == 0) {
            reject("x and y are 0");
            return;
        }

        if (est.timestampSeconds == lastTimestamp) {
            reject("stale - no new frame");
            return;
        }
        lastTimestamp = est.timestampSeconds;

        // publish pose to NT for AdvantageScope/Shuffleboard
        posePublisher.set(est.pose);

        StringBuilder tagIDs = new StringBuilder();
        if (est.rawFiducials != null) {
            for (int i = 0; i < est.rawFiducials.length; i++) {
                if (i > 0) tagIDs.append(", ");
                tagIDs.append(est.rawFiducials[i].id);
            }
        }
        SmartDashboard.putString("Vision/TagIDs", tagIDs.toString());
        // use Limelight's own stddevs instead of hardcoded values
        // layout: [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1yaw, MT2x, MT2y, MT2z, MT2roll,
        // MT2pitch, MT2yaw]
        var stddevs = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "stddevs");
        if (stddevs == null || stddevs.length < 8) {
            reject("missing stddevs");
            return;
        }

        drive.addVisionMeasurement(
                est.pose,
                est.timestampSeconds,
                VecBuilder.fill(stddevs[6], stddevs[7], Double.POSITIVE_INFINITY));

        // SmartDashboard.putBoolean("vision/measurementAccepted", true);
        // SmartDashboard.putString("vision/rejectReason", "");
        accept();
    }

    private void reject(String why) {
        SmartDashboard.putBoolean("Vision/measurementAccepted", false);
        SmartDashboard.putString("Vision/rejection", why);
        SmartDashboard.putString("Vision/TagIDs", "");
    }

    private void accept() {
        SmartDashboard.putBoolean("Vision/measurementAccepted", true);
        SmartDashboard.putString("Vision/rejection", "");
    }
}
