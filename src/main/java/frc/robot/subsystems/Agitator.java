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

public class Agitator extends SubsystemBase {

  private static final double kS = 0.1;
  private static final double kV = 0.12;
  private static final double kP = 0.0;
  private static final double kI = 0.0;

  private static final double kIdleRpm =  45.0;

  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  private final RelativeEncoder encoder;

  private double loggedSetpointRpm = 0.0;

  public Agitator(int id) {
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
  //   controller.leftBumper().whileTrue(agitator.idleCommand());
  // ---------------------------------------------------------------------------

  public Command idleCommand() {
    return run(() -> {
      loggedSetpointRpm = kIdleRpm;
      pid.setSetpoint(kIdleRpm, ControlType.kVelocity);
    }).withName("Agitator.idle");
  }

  public Command runAtRpm(double rpm) {
    return run(() -> {
      loggedSetpointRpm = rpm;
      pid.setSetpoint(rpm, ControlType.kVelocity);
    }).withName("Agitator.runAtRpm(" + rpm + ")");
  }

  public Command stopCommand() {
    return runOnce(() -> {
      loggedSetpointRpm = 0.0;
      pid.setSetpoint(0.0, ControlType.kVelocity);
    }).withName("Agitator.stop");
  }

  // Legacy — kept for compatibility
  @Deprecated
  public void setIdle(int rpm) {
    loggedSetpointRpm = rpm;
    pid.setReference(rpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    DogLog.log("Agitator/SetpointRPM",   loggedSetpointRpm);
    DogLog.log("Agitator/MeasuredRPM",   encoder.getVelocity());
    DogLog.log("Agitator/AppliedOutput", motor.getAppliedOutput());
    DogLog.log("Agitator/BusVoltage",    motor.getBusVoltage());
    DogLog.log("Agitator/StatorCurrent", motor.getOutputCurrent());
  }
}