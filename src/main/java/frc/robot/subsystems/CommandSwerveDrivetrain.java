package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final SwerveRequest.FieldCentric drive;

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SlewRateLimiter limiter = new SlewRateLimiter(1);

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private static final Rotation2d kRedAlliancePerspectiveRotation  = Rotation2d.fromDegrees(0);

    private boolean m_hasAppliedOperatorPerspective = false;

    // -----------------------------------------------------------------------
    // Vision seeding
    // On first valid Limelight measurement we hard-reset the pose estimator
    // so odometry starts from a real field position instead of (0, 0).
    // Without this, a large initial error causes the Kalman filter to reject
    // vision measurements because they disagree too much with odometry.
    // -----------------------------------------------------------------------
    private boolean m_hasSeededPose = false;

    private final StructPublisher<Pose2d> posePublisher;
    private final NetworkTable telemetryTable;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains  m_steerCharacterization       = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation    m_rotationCharacterization    = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds m_pathPlannerRequest = new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.FieldCentricFacingAngle m_aimRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(3.5, 0.0, 0.0);

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private final Limelight limelight;

    public CommandSwerveDrivetrain(
        SwerveRequest.FieldCentric driveCentric,
        SwerveDrivetrainConstants drivetrainConstants,
        Limelight limelight,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        this.drive = driveCentric;
        this.limelight = limelight;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("RealPose");
        posePublisher = telemetryTable.getStructTopic("Real Pose", Pose2d.struct).publish();

        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(
        SwerveRequest.FieldCentric driveCentric,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Limelight limelight,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.drive = driveCentric;
        this.limelight = limelight;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("RealPose");
        posePublisher = telemetryTable.getStructTopic("Real Pose", Pose2d.struct).publish();

        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(
        SwerveRequest.FieldCentric driveCentric,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        Limelight limelight,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        this.drive = driveCentric;
        this.limelight = limelight;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("RealPose");
        posePublisher = telemetryTable.getStructTopic("Real Pose", Pose2d.struct).publish();

        if (Utils.isSimulation()) startSimThread();
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(3.85, 0.0, 0.0),
                new PIDConstants(3.2,  0.0, 0.0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            },
            this
        );
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(m_pathPlannerRequest.withSpeeds(speeds));
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command runJoystickInputs(double x, double y, double rotation) {
        return run(() ->
            this.setControl(
                drive.withVelocityX(limiter.calculate(x))
                    .withVelocityY(limiter.calculate(y))
                    .withRotationalRate(limiter.calculate(rotation))
            )
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Command runAimingInputs(
            DoubleSupplier x,
            DoubleSupplier y,
            Supplier<Rotation2d> targetHeading) {
        return run(() ->
            this.setControl(
                m_aimRequest
                    .withVelocityX(x.getAsDouble())
                    .withVelocityY(y.getAsDouble())
                    .withTargetDirection(targetHeading.get())
            )
        );
    }

    @Override
    public void periodic() {
        // Apply alliance-relative forward direction once we know the alliance.
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // -----------------------------------------------------------------
        // Vision: seed the pose estimator on the first valid measurement.
        // This eliminates the large initial odometry error that would
        // otherwise prevent vision from correcting the estimate.
        // -----------------------------------------------------------------
        if (!m_hasSeededPose) {
            limelight.getRawEstimate(getPose()).ifPresent(estimate -> {
                // Hard-reset to the limelight's reported position.
                // Keep the current gyro heading so the reset feels smooth.
                Pose2d seedPose = new Pose2d(
                    estimate.pose.getTranslation(),
                    getPose().getRotation()
                );
                resetPose(seedPose);
                m_hasSeededPose = true;
            });
        }

        // Normal every-loop vision update.
        limelight.getMeasurement(getPose()).ifPresent(measurement ->
            addVisionMeasurement(
                measurement.poseEstimate.pose,
                measurement.poseEstimate.timestampSeconds,
                measurement.standardDeviations
            )
        );

        // Publish real pose for AdvantageScope.
        posePublisher.set(getPose());
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}