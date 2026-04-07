package frc.robot.util.singlemotortests;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SingleMotorVelocityPIDFSparkMaxTest extends SubsystemBase {
  private static final int kMotorCanId = 31;
  private static final boolean kInverted = true;

  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController cl = motor.getClosedLoopController();
  private final RelativeEncoder enc = motor.getEncoder();

  private final SparkMaxConfig cfg;

  private final LoggedTunableNumber enabled =
      new LoggedTunableNumber("SingleMotorTest/Enabled", 0.0);
  private final LoggedTunableNumber setpointRpm =
      new LoggedTunableNumber("SingleMotorTest/SetpointRPM", 0.0);

  private final LoggedTunableNumber useMaxMotionVel =
      new LoggedTunableNumber("SingleMotorTest/UseMAXMotionVelocity", 0.0);
  private final LoggedTunableNumber maxAccelRpmPerSec =
      new LoggedTunableNumber("SingleMotorTest/MaxAccelRPMperSec", 20000.0);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("SingleMotorTest/kP", 0.00006);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("SingleMotorTest/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("SingleMotorTest/kD", 0.0);

  // ***** kA only applied in MAXMotion modes. *****
  private final LoggedTunableNumber kS = new LoggedTunableNumber("SingleMotorTest/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("SingleMotorTest/kV", 0.00183);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("SingleMotorTest/kA", 0.0);

  private final LoggedTunableNumber minOut =
      new LoggedTunableNumber("SingleMotorTest/MinOut", -1.0);
  private final LoggedTunableNumber maxOut = new LoggedTunableNumber("SingleMotorTest/MaxOut", 1.0);

  private final int changeId = System.identityHashCode(this);

  public SingleMotorVelocityPIDFSparkMaxTest() {

    cfg = new SparkMaxConfig();

    cfg.idleMode(SparkMaxConfig.IdleMode.kCoast)
        .inverted(kInverted)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);

    applyConfig(ResetMode.kResetSafeParameters);
  }

  private void applyConfig(ResetMode resetMode) {

    System.out.println("kV : " + kV.get() + " Kp : " + kP);

    cfg.closedLoop
        .pid(kP.get(), kI.get(), kD.get(), ClosedLoopSlot.kSlot0)
        .outputRange(minOut.get(), maxOut.get(), ClosedLoopSlot.kSlot0);

    cfg.closedLoop
        .feedForward
        .kS(kS.get(), ClosedLoopSlot.kSlot0)
        .kV(kV.get(), ClosedLoopSlot.kSlot0);

    // During tuning: do NOT persist every change (flash wear + slow).
    motor.configure(cfg, resetMode, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        changeId,
        () -> applyConfig(ResetMode.kNoResetSafeParameters),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        minOut,
        maxOut,
        maxAccelRpmPerSec);

    if (enabled.get() > 0.5) {

      // System.out.println("enabled");
      double sp = setpointRpm.get();
      ControlType type =
          (useMaxMotionVel.get() > 0.5)
              ? ControlType.kMAXMotionVelocityControl
              : ControlType.kVelocity;

      cl.setSetpoint(sp, type, ClosedLoopSlot.kSlot0);
    } else {
      motor.stopMotor();
    }

    SmartDashboard.putNumber("SingleMotorTest/MeasuredRPM", enc.getVelocity());
    SmartDashboard.putNumber("SingleMotorTest/AppliedOutput", motor.getAppliedOutput());
    SmartDashboard.putNumber("SingleMotorTest/BusVoltage", motor.getBusVoltage());
    SmartDashboard.putNumber("SingleMotorTest/MeasuredRPM", enc.getVelocity());
    SmartDashboard.putNumber("SingleMotorTest/SetpointRPM", setpointRpm.get());
    SmartDashboard.putNumber("SingleMotorTest/UseMAXMotionVelocity", useMaxMotionVel.get());

    Logger.recordOutput("SingleMotorTest/MeasuredRPM", enc.getVelocity());
  }
}
