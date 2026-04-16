package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX motor;
  private final TalonFXConfiguration cfg;

  private double speed = 0.0;
  private double oldSpeed = 0.0;

  /** Creates a new Indexer. */
  public Indexer(int id) {
    motor = new TalonFX(id);
    
    cfg = new TalonFXConfiguration();
    
    MotorOutputConfigs motorOut = cfg.MotorOutput;
    motorOut.Inverted = InvertedValue.Clockwise_Positive;
    motorOut.NeutralMode = NeutralModeValue.Coast;
    
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor.getConfigurator().apply(cfg);
  }

  public void intake() {
    speed = 0.70;
  }

  public void outake() {
    speed = -0.30;
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
  }
}