package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static final double kS = 0.1;
  private static final double kV = 0.12;
  private static final double kP = 0.0;
  private static final double kI = 0.0;

  private static final double kIntakeRpm = -300.0;
  private static final double kEjectRpm  =  100.0;

  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  private final RelativeEncoder encoder;

  private double loggedSetpointRpm = 0.0;

  public Intake(int id) {
    motor   = new SparkMax(id, MotorType.kBrushless);
    pid     = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.inverted(true);
    config.closedLoop.feedForward.kS(kS);
    config.closedLoop.feedForward.kV(kV);
    config.closedLoop.p(kP);
    config.closedLoop.i(kI);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ---------------------------------------------------------------------------
  // Commands — use these from RobotContainer
  //   controller.rightBumper().whileTrue(intake.intakeCommand());
  //   controller.leftBumper().whileTrue(intake.ejectCommand());
  // ---------------------------------------------------------------------------

  public Command intakeCommand() {
    return run(() -> {
      loggedSetpointRpm = kIntakeRpm;
      pid.setSetpoint(kIntakeRpm, ControlType.kVelocity);
    }).withName("Intake.intake");
  }

  public Command ejectCommand() {
    return run(() -> {
      loggedSetpointRpm = kEjectRpm;
      pid.setSetpoint(kEjectRpm, ControlType.kVelocity);
    }).withName("Intake.eject");
  }

  public Command runAtRpm(double rpm) {
    return run(() -> {
      loggedSetpointRpm = rpm;
      pid.setSetpoint(rpm, ControlType.kVelocity);
    }).withName("Intake.runAtRpm(" + rpm + ")");
  }

  public Command stopCommand() {
    return runOnce(() -> {
      loggedSetpointRpm = 0.0;
      pid.setSetpoint(0.0, ControlType.kVelocity);
    }).withName("Intake.stop");
  }

  @Deprecated
  public void on(int rpm) {
    loggedSetpointRpm = rpm;
    pid.setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    DogLog.log("Intake/SetpointRPM",   loggedSetpointRpm);
    DogLog.log("Intake/MeasuredRPM",   encoder.getVelocity());
    DogLog.log("Intake/AppliedOutput", motor.getAppliedOutput());
    DogLog.log("Intake/BusVoltage",    motor.getBusVoltage());
    DogLog.log("Intake/StatorCurrent", motor.getOutputCurrent());
  }
}