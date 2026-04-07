package frc.robot.util.singlemotortests;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SingleMotorVelocityPIDFKrakenTest extends SubsystemBase {
  private static final int kMotorCanId = 14;
  private static final boolean kInverted = false;

  private final TalonFX motor = new TalonFX(kMotorCanId);

  // Reuse control request objects to avoid GC pressure — Phoenix 6 best practice.
  // Phoenix 6 velocity units are ROTATIONS PER SECOND (RPS), not RPM.
  // We store setpoints in RPM for readability and convert on use: RPS = RPM / 60.
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final MotionMagicVelocityVoltage mmVelocityRequest =
      new MotionMagicVelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final LoggedTunableNumber enabled =
      new LoggedTunableNumber("SingleMotorTest/Enabled", 0.0);
  private final LoggedTunableNumber setpointRpm =
      new LoggedTunableNumber("SingleMotorTest/SetpointRPM", 0.0);

  private final LoggedTunableNumber useMotionMagicVel =
      new LoggedTunableNumber("SingleMotorTest/UseMotionMagicVelocity", 0.0);

  // MotionMagic acceleration in RPS/s (rotations per second, per second).
  // To convert from RPM/s: divide by 60.  Default 20000 RPM/s → ~333 RPS/s.
  private final LoggedTunableNumber maxAccelRps =
      new LoggedTunableNumber("SingleMotorTest/MaxAccelRPSperSec", 333.0);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("SingleMotorTest/kP", 0.0432);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("SingleMotorTest/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("SingleMotorTest/kD", 0.0);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("SingleMotorTest/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("SingleMotorTest/kV", 0.1098);
  // kA is only meaningful when UseMotionMagicVelocity = 1.
  private final LoggedTunableNumber kA = new LoggedTunableNumber("SingleMotorTest/kA", 0.0);

  // Peak duty-cycle output limits (-1.0 to 1.0).
  private final LoggedTunableNumber minOut =
      new LoggedTunableNumber("SingleMotorTest/MinOut", -1.0);
  private final LoggedTunableNumber maxOut =
      new LoggedTunableNumber("SingleMotorTest/MaxOut", 1.0);

  private final int changeId = System.identityHashCode(this);

  public SingleMotorVelocityPIDFKrakenTest() {
    applyConfig();
  }

  private void applyConfig() {
    System.out.println("kV: " + kV.get() + "  kP: " + kP.get());

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Motor output
    MotorOutputConfigs motorOut = cfg.MotorOutput;
    motorOut.Inverted =
        kInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorOut.NeutralMode = NeutralModeValue.Coast;
    motorOut.PeakForwardDutyCycle = maxOut.get();
    motorOut.PeakReverseDutyCycle = minOut.get();

    // Closed-loop gains (Slot 0)
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = kP.get();
    slot0.kI = kI.get();
    slot0.kD = kD.get();
    slot0.kS = kS.get();
    slot0.kV = kV.get();
    slot0.kA = kA.get();

    // MotionMagic velocity profile (used when UseMotionMagicVelocity = 1)
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicAcceleration = maxAccelRps.get(); // RPS/s
    // MotionMagicJerk = 0 → no jerk limiting (smooth but uncapped)

    // Current limits — Kraken X60 is rated for 120 A peak; 40 A is a reasonable test limit.
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Voltage compensation — Phoenix 6 handles this internally via FOC; no explicit setting needed.

    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        changeId,
        this::applyConfig,
        kP, kI, kD, kS, kV, kA,
        minOut, maxOut, maxAccelRps);

    if (enabled.get() > 0.5) {
      // Convert RPM → RPS for Phoenix 6.
      double setpointRps = setpointRpm.get() / 60.0;

      if (useMotionMagicVel.get() > 0.5) {
        motor.setControl(mmVelocityRequest.withVelocity(setpointRps));
      } else {
        motor.setControl(velocityRequest.withVelocity(setpointRps));
      }
    } else {
      motor.setControl(neutralRequest);
    }

    // Publish telemetry — getVelocity() returns RPS; multiply by 60 for RPM readout.
    double measuredRpm = motor.getVelocity().getValueAsDouble() * 60.0;
    double appliedOutput = motor.getDutyCycle().getValueAsDouble();
    double busVoltage = motor.getSupplyVoltage().getValueAsDouble();

    SmartDashboard.putNumber("SingleMotorTest/MeasuredRPM", measuredRpm);
    SmartDashboard.putNumber("SingleMotorTest/AppliedOutput", appliedOutput);
    SmartDashboard.putNumber("SingleMotorTest/BusVoltage", busVoltage);
    SmartDashboard.putNumber("SingleMotorTest/SetpointRPM", setpointRpm.get());
    SmartDashboard.putNumber("SingleMotorTest/UseMotionMagicVelocity", useMotionMagicVel.get());

    Logger.recordOutput("SingleMotorTest/MeasuredRPM", measuredRpm);
  }
}