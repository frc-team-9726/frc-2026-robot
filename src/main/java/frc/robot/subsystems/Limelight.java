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

        // ---------------------------------------------------------------
        // Camera pose relative to robot center.
        // Adjust these numbers to match your physical mounting!
        //   forward (+x), left (+y), up (+z) in meters
        //   roll, pitch, yaw in degrees
        //
        // yaw = 0   → camera faces forward
        // yaw = 180 → camera faces rearward
        // yaw = 90  → camera faces LEFT — change if yours faces forward!
        // ---------------------------------------------------------------
        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            0.3937, 0.2032, 0.349, // forward, left, up (meters)
             0.0,    22.5,   0.0  // roll, pitch, yaw (degrees)
        );

        // Mode 0 = external gyro via SetRobotOrientation().
        // Do NOT use the internal IMU (mode 1) at the same time as
        // SetRobotOrientation() — they conflict and produce jitter.
        LimelightHelpers.SetIMUMode(name, 0);
    }

    /**
     * Returns a vision measurement fused with the current gyro heading,
     * or empty if no tags are visible or the estimate is untrustworthy.
     *
     * @param currentRobotPose  Latest pose from the drivetrain pose estimator.
     *                          Used only for its rotation (gyro heading).
     */
    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Supply the robot's current gyro yaw to MegaTag2 every loop.
        LimelightHelpers.SetRobotOrientation(
            name,
            currentRobotPose.getRotation().getDegrees() - 90,
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

        // Fuse: keep the limelight's XY translation but trust the gyro
        // for rotation (heading) — MegaTag2 already does this internally,
        // but we make it explicit so the downstream filter sees it clearly.
        Pose2d fusedPose = new Pose2d(
            mt2.pose.getTranslation(),
            currentRobotPose.getRotation()
        );

        // Scale translational std devs with distance so nearby tags pull
        // the estimate harder than far-away ones.
        // rotational stddev = 9999 → never update heading from vision
        // (gyro is far more accurate for heading).
        double xyStdDev = 0.1 + mt2.avgTagDist * 0.002;
        final Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 9999.0);

        // Publish telemetry for AdvantageScope / Shuffleboard.
        fusedPosePublisher.set(fusedPose);
        mt2PosePublisher.set(mt2.pose);

        currentPose = mt2.pose;

        mt2.pose = fusedPose;
        return Optional.of(new Measurement(mt2, stdDevs));
    }

    /** Returns the raw MegaTag2 estimate without any filtering, for seeding. */
    public Optional<PoseEstimate> getRawEstimate(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(
            name,
            currentRobotPose.getRotation().getDegrees() - 90,
            0, 0, 0, 0, 0
        );
        PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt2 == null || mt2.tagCount == 0) return Optional.empty();
        return Optional.of(mt2);
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

    public Pose2d getPose2d() {
        return currentPose;
    }
}