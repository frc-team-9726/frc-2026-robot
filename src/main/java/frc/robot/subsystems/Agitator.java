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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  private final RelativeEncoder encoder;
  private double setpoint = 0;

  /** Creates a new Indexer. */
  public Agitator(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);
    pid = motor.getClosedLoopController();
    
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);

    config.inverted(true);
    config.closedLoop.feedForward.kS(0.1);
    config.closedLoop.feedForward.kV(0.12);
    config.closedLoop.p(0);
    config.closedLoop.i(0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public void setIdle(int setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pid.setSetpoint(setpoint, ControlType.kVelocity);
    DogLog.log("indexer/setpoint", setpoint);
    DogLog.log("indexer/speed", encoder.getVelocity());
  }
}

