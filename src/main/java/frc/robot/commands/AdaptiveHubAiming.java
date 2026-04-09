package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.util.FieldConstants;
import frc.robot.util.ShotTable;
import frc.robot.util.ShotTable.ShotSetpoint;
import frc.robot.util.ShotTimeTable;

public class AdaptiveHubAiming extends Command {

  private static final int    LOOKAHEAD_ITERS       = 6;
  private static final double TOF_EPSILON_SEC       = 0.001;
  private static final double MAX_LEAD_TIME_SEC     = 2.0;
  private static final double SHOT_RELEASE_DELAY_SEC = 0.05;
  private static final double TOF_TABLE_MIN_M       = 0.0;
  private static final double TOF_TABLE_MAX_M       = 15.0;

  private final Flywheel flywheel;
  private final CommandSwerveDrivetrain drive;

  private final boolean  isBlue;

  private double fieldAimAngleRad   = 0.0;
  private double aimPathDistMeters  = 0.0;

  private enum Target { HUB, OUTPOST, DEPOT, NONE }
  private Target targetChoice = Target.HUB;

  public AdaptiveHubAiming(Flywheel flywheel, CommandSwerveDrivetrain drive, boolean isBlue) {
    this.flywheel = flywheel;
    this.drive    = drive;
    this.isBlue   = isBlue;
    addRequirements(flywheel);
  }

  @Override
  public void execute() {
    Pose2d        robotPose   = drive.getPose();
    ChassisSpeeds robotSpeeds = drive.getRobotRelativeSpeeds();

    updateTargetChoice(robotPose);
    Translation2d targetField = getTargetTranslation(targetChoice);

    AimSolution solution = solveAim(robotPose, robotSpeeds, targetField);

    fieldAimAngleRad  = solution.fieldAimAngleRad;
    aimPathDistMeters = solution.aimPathDistanceMeters;

    flywheel.setSetpointRpm(solution.shotSetpoint.shooterSpeed());

    logSolution(solution);
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double getFieldAimAngleRad() {
    return fieldAimAngleRad;
  }

  public Rotation2d getFieldAimRotation() {
    return Rotation2d.fromRadians(fieldAimAngleRad);
  }

  public double getAimPathDistanceMeters() {
    return aimPathDistMeters;
  }

  private AimSolution solveAim(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation2d targetField) {

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    Translation2d pivotFieldVelocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Translation2d pivotFieldPosition = robotPose.getTranslation();

    Translation2d releasePivotFieldPosition =
        addScaled(pivotFieldPosition, pivotFieldVelocity, SHOT_RELEASE_DELAY_SEC);

    double shotDistanceMeters    = releasePivotFieldPosition.getDistance(targetField);
    double aimPathDistanceMeters = shotDistanceMeters;
    double tofSeconds            = flightTimeSecondsSafe(aimPathDistanceMeters);
    Translation2d aimPointField  = targetField;

    for (int i = 0; i < LOOKAHEAD_ITERS; i++) {
      Translation2d newAimPoint        = addScaled(targetField, pivotFieldVelocity, -tofSeconds);
      double        newAimPathDistance = releasePivotFieldPosition.getDistance(newAimPoint);
      double        newTof             = flightTimeSecondsSafe(newAimPathDistance);

      aimPointField        = newAimPoint;
      aimPathDistanceMeters = newAimPathDistance;

      boolean converged = Math.abs(newTof - tofSeconds) < TOF_EPSILON_SEC;
      tofSeconds = newTof;
      if (converged) break;
    }

    ShotSetpoint shotSetpoint = ShotTable.get(aimPathDistanceMeters);

    double fieldAimAngleRad =
        Math.atan2(
            aimPointField.getY() - releasePivotFieldPosition.getY(),
            aimPointField.getX() - releasePivotFieldPosition.getX());

    return new AimSolution(
        fieldAimAngleRad,
        shotDistanceMeters,
        aimPathDistanceMeters,
        tofSeconds,
        shotSetpoint,
        pivotFieldPosition,
        releasePivotFieldPosition,
        aimPointField,
        pivotFieldVelocity);
  }

  private void updateTargetChoice(Pose2d robotPose) {
    double xM = robotPose.getX();
    double normalizedX =
        isBlue ? xM : (Units.inchesToMeters(FieldConstants.FIELD_LENGTH_INCHES) - xM);

    if (normalizedX < Units.inchesToMeters(165)) {
      targetChoice = Target.HUB;
      DogLog.log("AimDebug/FieldZone", "hub");
    } else if (normalizedX < Units.inchesToMeters(200)) {
      targetChoice = Target.NONE;
      DogLog.log("AimDebug/FieldZone", "close trench zone");
    } else {
      boolean inOutpostSide =
          isBlue
              ? robotPose.getY() < Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2.0)
              : robotPose.getY() >= Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2.0);

      if (inOutpostSide) {
        targetChoice = Target.OUTPOST;
        DogLog.log("AimDebug/FieldZone", "outpost");
      } else {
        targetChoice = Target.DEPOT;
        DogLog.log("AimDebug/FieldZone", "depot");
      }
    }
  }

  private Translation2d getTargetTranslation(Target target) {
    return switch (target) {
      case DEPOT   -> (isBlue ? FieldConstants.BLUE_DEPOT_POSE3D   : FieldConstants.RED_DEPOT_POSE3D)
                          .getTranslation().toTranslation2d();
      case OUTPOST -> (isBlue ? FieldConstants.BLUE_OUTPOST_POSE3D : FieldConstants.RED_OUTPOST_POSE3D)
                          .getTranslation().toTranslation2d();
      default      -> (isBlue ? FieldConstants.BLUE_HUB_POSE3D    : FieldConstants.RED_HUB_POSE3D)
                          .getTranslation().toTranslation2d();
    };
  }

  private Translation2d addScaled(Translation2d base, Translation2d delta, double scale) {
    return new Translation2d(
        base.getX() + delta.getX() * scale,
        base.getY() + delta.getY() * scale);
  }

  private double flightTimeSecondsSafe(double distanceMeters) {
    double clamped = MathUtil.clamp(distanceMeters, TOF_TABLE_MIN_M, TOF_TABLE_MAX_M);
    return MathUtil.clamp(ShotTimeTable.getFlightTimeSeconds(clamped), 0.0, MAX_LEAD_TIME_SEC);
  }

  // ── Logging ───────────────────────────────────────────────────────────────

  private void logSolution(AimSolution s) {
    DogLog.log("AimDebug/TOFSeconds",         s.tofSeconds);
    DogLog.log("AimDebug/ShotDistMeters",     s.shotDistanceMeters);
    DogLog.log("AimDebug/AimPathDistMeters",  s.aimPathDistanceMeters);
    DogLog.log("AimDebug/FieldAimAngleDeg",   Math.toDegrees(s.fieldAimAngleRad));
    DogLog.log("AimDebug/FlywheelSetpointRPM", s.shotSetpoint.shooterSpeed());
    DogLog.log("AimDebug/PivotVelX",          s.pivotFieldVelocity.getX());
    DogLog.log("AimDebug/PivotVelY",          s.pivotFieldVelocity.getY());
    DogLog.log("AimDebug/PivotPose",
        new Pose2d(s.pivotFieldPosition, new Rotation2d()));
    DogLog.log("AimDebug/ReleasePose",
        new Pose2d(s.releasePivotFieldPosition, new Rotation2d()));
    DogLog.log("AimDebug/AimPose",
        new Pose2d(s.releasePivotFieldPosition, Rotation2d.fromRadians(s.fieldAimAngleRad)));
    DogLog.log("AimDebug/VirtualTarget",
        new Pose2d(s.aimPointField, new Rotation2d()));
  }

  // ── Result record ─────────────────────────────────────────────────────────

  private static final class AimSolution {
    final double        fieldAimAngleRad;
    final double        shotDistanceMeters;
    final double        aimPathDistanceMeters;
    final double        tofSeconds;
    final ShotSetpoint  shotSetpoint;
    final Translation2d pivotFieldPosition;
    final Translation2d releasePivotFieldPosition;
    final Translation2d aimPointField;
    final Translation2d pivotFieldVelocity;

    AimSolution(
        double        fieldAimAngleRad,
        double        shotDistanceMeters,
        double        aimPathDistanceMeters,
        double        tofSeconds,
        ShotSetpoint  shotSetpoint,
        Translation2d pivotFieldPosition,
        Translation2d releasePivotFieldPosition,
        Translation2d aimPointField,
        Translation2d pivotFieldVelocity) {
      this.fieldAimAngleRad          = fieldAimAngleRad;
      this.shotDistanceMeters        = shotDistanceMeters;
      this.aimPathDistanceMeters     = aimPathDistanceMeters;
      this.tofSeconds                = tofSeconds;
      this.shotSetpoint              = shotSetpoint;
      this.pivotFieldPosition        = pivotFieldPosition;
      this.releasePivotFieldPosition = releasePivotFieldPosition;
      this.aimPointField             = aimPointField;
      this.pivotFieldVelocity        = pivotFieldVelocity;
    }
  }
}