// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  private double setpoint = 0;

  /** Creates a new Intake. */
  public Intake(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);
    pid = motor.getClosedLoopController();
    
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.inverted(true);

    config.closedLoop.feedForward.kS(0.1);
    config.closedLoop.feedForward.kV(0.12);
    config.closedLoop.p(0);
    config.closedLoop.i(0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void on(int setpoint) {
    this.setpoint = setpoint;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pid.setSetpoint(setpoint, ControlType.kVelocity);
  }
}
