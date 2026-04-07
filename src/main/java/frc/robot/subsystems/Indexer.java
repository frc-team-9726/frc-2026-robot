package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private static final boolean kInverted = false;
  private static final int kCurrentLimitAmps = 40;

  private static final double kS = 0.0;
  private static final double kV = 0.1098;
  private static final double kA = 0.0;
  private static final double kP = 0.0432;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kIdleRps  = -2000.0 / 60.0;
  private static final double kJamRps   =   2000.0 / 60.0;

  private final TalonFX motor;

  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private double loggedSetpointRps = 0.0;

  public Indexer(int id) {
    motor = new TalonFX(id);
    applyConfig();
  }

  private void applyConfig() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotorOutputConfigs motorOut = cfg.MotorOutput;
    motorOut.Inverted =
        kInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorOut.NeutralMode = NeutralModeValue.Coast;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    cfg.CurrentLimits.SupplyCurrentLimit = kCurrentLimitAmps;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(cfg);
  }

  // ---------------------------------------------------------------------------
  // Example:
  //   controller.leftBumper().whileTrue(indexer.idleCommand());
  //   controller.rightBumper().whileTrue(indexer.jamCommand());
  // ---------------------------------------------------------------------------

  public Command idleCommand() {
    return run(() -> {
      loggedSetpointRps = kIdleRps;
      motor.setControl(velocityRequest.withVelocity(kIdleRps));
    }).withName("Indexer.idle");
  }

  public Command jamCommand() {
    return run(() -> {
      loggedSetpointRps = kJamRps;
      motor.setControl(velocityRequest.withVelocity(kJamRps));
    }).withName("Indexer.jam");
  }

  public Command runAtRps(double rps) {
    return run(() -> {
      loggedSetpointRps = rps;
      motor.setControl(velocityRequest.withVelocity(rps));
    }).withName("Indexer.runAtRps(" + rps + ")");
  }

  public Command stopCommand() {
    return runOnce(() -> {
      loggedSetpointRps = 0.0;
      motor.setControl(neutralRequest);
    }).withName("Indexer.stop");
  }

  @Deprecated
  public void setIdle() { loggedSetpointRps = kIdleRps; motor.setControl(velocityRequest.withVelocity(kIdleRps)); }

  @Deprecated
  public void jam()     { loggedSetpointRps = kJamRps;  motor.setControl(velocityRequest.withVelocity(kJamRps));  }

  @Deprecated
  public void off()     { loggedSetpointRps = 0.0;      motor.setControl(neutralRequest); }

  @Override
  public void periodic() {
    double measuredRps = motor.getVelocity().getValueAsDouble();

    DogLog.log("Indexer/SetpointRPS",    loggedSetpointRps);
    DogLog.log("Indexer/MeasuredRPS",    measuredRps);
    DogLog.log("Indexer/AppliedOutput",  motor.getDutyCycle().getValueAsDouble());
    DogLog.log("Indexer/BusVoltage",     motor.getSupplyVoltage().getValueAsDouble());
    DogLog.log("Indexer/StatorCurrent",  motor.getStatorCurrent().getValueAsDouble());
  }
}