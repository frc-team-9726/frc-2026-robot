package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig sparkConfig;

  private double speed = 0.0;
  private double oldSpeed = 0.0;

  /** Creates a new Indexer. */
  public Intake(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);
    
    sparkConfig = new SparkMaxConfig();

    sparkConfig
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    sparkConfig.signals.appliedOutputPeriodMs(20);
    sparkConfig.inverted(true);

    motor.configure(
        sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    speed = 0.4;
  }

  public void outake() {
    speed = -0.4;
  }

  public void stop() {
    speed = 0.0;
  }

  @Override
  public void periodic() {
    if (speed != oldSpeed) {
      oldSpeed = speed;
      motor.set(oldSpeed);
    }
    DogLog.log("indexer/outputCurrent", motor.getOutputCurrent());
    DogLog.log("indexer/speed", motor.getEncoder().getVelocity());
  }
}