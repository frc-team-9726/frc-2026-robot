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

public class Flywheel extends SubsystemBase {

  private static final boolean kInverted = false;
  private static final int kCurrentLimitAmps = 40;

  private static final double kS = 0.0;    // [V]        — overcome static friction
  private static final double kV = 0.1098; // [V / RPS]  — steady-state (main FF term)
  private static final double kA = 0.0;    // [V / RPS²] — acceleration FF (optional)
  private static final double kP = 0.0432; // [V / RPS]  — proportional
  private static final double kI = 0.0;    // [V / (RPS·s)]
  private static final double kD = 0.0;    // [V·s / RPS]

  private final TalonFX motor;

  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private double loggedSetpointRpm = 0.0;

  public Flywheel(int id) {
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

  public Command spinAtRpm(double rpm) {
    return run(() -> {
      loggedSetpointRpm = rpm;
      motor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }).withName("Flywheel.spinAtRpm(" + rpm + ")");
  }

  public Command spinAtRpm(java.util.function.DoubleSupplier rpmSupplier) {
    return run(() -> {
      double rpm = rpmSupplier.getAsDouble();
      loggedSetpointRpm = rpm;
      motor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }).withName("Flywheel.spinAtRpm(supplier)");
  }

  public Command stopCommand() {
    return runOnce(() -> {
      loggedSetpointRpm = 0.0;
      motor.setControl(neutralRequest);
    }).withName("Flywheel.stop");
  }

  public boolean atSetpoint(double setpointRpm, double toleranceRpm) {
    double measuredRpm = motor.getVelocity().getValueAsDouble() * 60.0;
    return Math.abs(measuredRpm - setpointRpm) < toleranceRpm;
  }

  @Override
  public void periodic() {
    double measuredRpm = motor.getVelocity().getValueAsDouble() * 60.0;

    DogLog.log("Flywheel/SetpointRPM", loggedSetpointRpm);
    DogLog.log("Flywheel/MeasuredRPM", measuredRpm);
    DogLog.log("Flywheel/AppliedOutput", motor.getDutyCycle().getValueAsDouble());
    DogLog.log("Flywheel/BusVoltage", motor.getSupplyVoltage().getValueAsDouble());
    DogLog.log("Flywheel/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
  }
}