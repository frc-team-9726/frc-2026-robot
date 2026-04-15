package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;

    private Pose2d currentPose = new Pose2d();

    private final StructPublisher<Pose2d> fusedPosePublisher;
    private final StructPublisher<Pose2d> mt2PosePublisher;

    // Maximum distance (meters) at which we trust a vision measurement.
    // Tags seen from very far away produce noisy estimates.
    private static final double MAX_TRUSTED_TAG_DIST = 5.0;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault()
                .getTable("Limelight/" + name);

        fusedPosePublisher = telemetryTable
                .getStructTopic("Fused Pose", Pose2d.struct).publish();
        mt2PosePublisher = telemetryTable
                .getStructTopic("MT2 Raw Pose", Pose2d.struct).publish();

        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            -0.146,  // forward  (negative = backward)
            0.392,  // side     (negative = right)
             0.343,   // up
             0.0,     // roll
             22.5,    // pitch
            -90.0     // yaw: camera faces right side of robot
        );

        // Mode 0 = external gyro via SetRobotOrientation().
        LimelightHelpers.SetIMUMode(name, 0);
    }

    /**
     * Returns a vision measurement fused with the current gyro heading,
     * or empty if no tags are visible or the estimate is untrustworthy.
     *
     * @param currentRobotPose Latest pose from the drivetrain pose estimator.
     *                         Used to supply the gyro heading to MegaTag2.
     */
    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Supply the true gyro yaw — no offset needed because the camera
        // pose yaw is now set correctly above.
        LimelightHelpers.SetRobotOrientation(
            name,
            currentRobotPose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        final PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        // Reject empty or tagless results.
        if (mt2 == null || mt2.tagCount == 0) {
            return Optional.empty();
        }

        // Reject estimates from very far tags — they're too noisy to help.
        if (mt2.avgTagDist > MAX_TRUSTED_TAG_DIST) {
            return Optional.empty();
        }

        // Fuse: use MT2's XY translation (computed using correct camera geometry
        // now) and gyro rotation. MT2 already bakes in the gyro heading, but we
        // enforce it here explicitly so the downstream Kalman filter is never
        // asked to correct heading from vision (gyro wins for rotation).
        Pose2d fusedPose = new Pose2d(
            mt2.pose.getTranslation(),
            currentRobotPose.getRotation()
        );

        // Scale translational std devs with distance — nearby tags pull harder.
        // Rotational stddev = 9999 → never update heading from vision.
        double xyStdDev = 0.1 + mt2.avgTagDist * 0.002;
        final Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 9999.0);

        // Publish telemetry for AdvantageScope / Shuffleboard.
        fusedPosePublisher.set(fusedPose);
        mt2PosePublisher.set(mt2.pose);

        currentPose = fusedPose;

        mt2.pose = fusedPose;
        return Optional.of(new Measurement(mt2, stdDevs));
    }

    /**
     * Returns the raw MegaTag2 estimate without filtering, used for initial
     * pose seeding before odometry has a good reference.
     */
    public Optional<PoseEstimate> getRawEstimate(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(
            name,
            currentRobotPose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );
        PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt2 == null || mt2.tagCount == 0) return Optional.empty();
        return Optional.of(mt2);
    }

    /**
     * Returns the latest fused pose (MT2 XY + gyro heading).
     * Prefer drive.getPose() in commands that have drivetrain access,
     * as that pose is continuously corrected by both odometry and vision.
     */
    public Pose2d getPose2d() {
        return currentPose;
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate,
                           Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}