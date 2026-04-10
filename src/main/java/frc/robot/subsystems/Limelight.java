package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();

        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            -5.75, -15.75, 13.75, // x, y, z translation in meters
            0.0, 22.5, 90.0  // roll, pitch, yaw rotation in degrees
        );
        LimelightHelpers.SetIMUMode(name, 1);
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(
            name,
            // currentRobotPose.getRotation().getDegrees(),
            currentRobotPose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        final PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (mt1 == null || mt2 == null
                || mt1.tagCount == 0
                || mt2.tagCount == 0) {
            return Optional.empty();
        }

        Pose2d fusedPose = new Pose2d(
            mt2.pose.getTranslation(),
            mt1.pose.getRotation()
        );

        // For now these values tune later
        final Matrix<N3, N1> stdDevs = VecBuilder.fill(0.7, 0.7, 9999.0);

        posePublisher.set(fusedPose);

        mt2.pose = fusedPose;
        return Optional.of(new Measurement(mt2, stdDevs));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}