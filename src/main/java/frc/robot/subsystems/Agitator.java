package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig sparkConfig;

  private double speed = 0.0;
  private double oldSpeed = 0.0;

  /** Creates a new Indexer. */
  public Agitator(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);
    
    sparkConfig = new SparkMaxConfig();

    sparkConfig
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .smartCurrentLimit(40);
    sparkConfig.signals.appliedOutputPeriodMs(20);
    sparkConfig.inverted(true);

    motor.configure(
        sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    speed = 0.3;
  }

  public void outake() {
    speed = -0.25;
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
    DogLog.log("agitator/outputCurrent", motor.getOutputCurrent());
    DogLog.log("agitator/speed", motor.getEncoder().getVelocity());
  }
}