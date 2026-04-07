// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX motor;
  private double setpoint = 0;
  private StatusSignal<AngularVelocity> velSignal;
  /** Creates a new Indexer. */
  public Indexer(int id) {
    motor = new TalonFX(id);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    motor.getConfigurator().apply(slot0Configs);

    velSignal = motor.getVelocity();
  }
  //   motor = new SparkMax(id, MotorType.kBrushless);
  //   pid = motor.getClosedLoopController();
    
  //   encoder = motor.getEncoder();

  //   SparkMaxConfig config = new SparkMaxConfig();
  //   config.smartCurrentLimit(40);

  //   config.inverted(true);
  //   config.closedLoop.feedForward.kS(0.1);
  //   config.closedLoop.feedForward.kV(0.12);
  //   config.closedLoop.p(0);
  //   config.closedLoop.i(0);
  //   motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }


  public void setIdle() {
    setpoint = -400   ;
  }
  public void jam(){
    setpoint = 50 ;
  }
  public void off() {
    setpoint = 0 ;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final VelocityVoltage m_request = new VelocityVoltage(setpoint).withSlot(0);
    // final VelocityVoltage m_request = new VelocityVoltage(setpoint).withSlot(0);
    motor.setControl(m_request);

    velSignal.refresh();
    DogLog.log("indexer/setpoint", setpoint);
    DogLog.log("indexer/speed", velSignal.getValue());    
  }
}

