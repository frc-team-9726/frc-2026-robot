// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.util.singlemotortests.SingleMotorVelocityPIDFKrakenTest;

public class RobotContainer {
    // private final SingleMotorVelocityPIDFKrakenTest singleMotorVelocityPIDFKrakenTest;

    private final Pigeon2 pigeon2;
    private final Limelight limelight = new Limelight(""); 
    private final Flywheel flywheel = new Flywheel(14);
    private final Indexer indexer = new Indexer(20);
    private final Agitator agitator = new Agitator(4);
    private final Intake intake = new Intake(2);
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    //binds
    private final Joystick joystick = new Joystick(1);
    private final Joystick joystick2 = new Joystick(0);
    private final Trigger indexerButton = new JoystickButton(joystick,1);
    private final Trigger intakeButton = new JoystickButton(joystick,3);
    private final Trigger intakeEctButton = new JoystickButton(joystick, 4); 
    private final Trigger indexerJam = new JoystickButton(joystick, 5);

    private final Trigger gyroResetButton = new JoystickButton(joystick, 8);


    // private final Trigger brake = new JoystickButton(joystick,7);
    private final Trigger flywheel1trButton = new JoystickButton(joystick2, 1);
    private final Trigger flywheel2trButton = new JoystickButton(joystick2, 2);
    private final Trigger flywheel3trButton = new JoystickButton(joystick2, 3);
    private final Trigger flywheel4trButton = new JoystickButton(joystick2, 4);
    private final Trigger flywheel5trButton = new JoystickButton(joystick2, 5);
    private final Trigger flywheel6trButton = new JoystickButton(joystick2, 6);
    private final Trigger flywheel7trButton = new JoystickButton(joystick2, 7);
    private final Trigger flywheel8trButton = new JoystickButton(joystick2, 8);
    private final Trigger flywheel9trButton = new JoystickButton(joystick2,9);
    private final Trigger flywheel10trButton = new JoystickButton(joystick2, 10);
    private final Trigger flywheel11trButton = new JoystickButton(joystick2, 11);
    private final Trigger flywheel12trButton = new JoystickButton(joystick2, 12);
    private final Trigger flywheel13trButton = new JoystickButton(joystick2, 13);
    private final Trigger flywheel14trButton = new JoystickButton(joystick2,14);
    private final Trigger flywheel15trButton = new JoystickButton(joystick2, 15);
    private final Trigger OFFButton = new JoystickButton(joystick2, 20);




    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(drive);

    public RobotContainer() {
        pigeon2 = new Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus);
        // singleMotorVelocityPIDFKrakenTest = new SingleMotorVelocityPIDFKrakenTest();
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // drivetrain.runJoystickInputs(-joystick.getY()/2, -joystick.getX()/2, -joystick2.getZ()/4)
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getY()/2 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getX()/2 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate((-joystick.getZ())) // Drive counterclockwise with negative X (left)
                    ///8 * MaxAngularRate
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
             
        flywheel1trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3200))); //button 15
        flywheel2trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3400))); //button 10
        flywheel3trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3600))); // button 5
        flywheel4trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3800))); //button 15
        flywheel5trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4000))); //button 10
        flywheel6trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4200))); // button 5
        flywheel7trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4400))); //button 15
        flywheel8trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4600))); //button 10
        flywheel9trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4800))); // button 5
        flywheel10trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5000))); // button 5
        flywheel11trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5200))); // button 5
        flywheel12trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5400))); //button 15
        flywheel13trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5600))); //button 10
        flywheel14trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5800))); // button 5
        flywheel15trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(6000))); // button 5


        gyroResetButton.onTrue(Commands.runOnce(
            () -> {
                pigeon2.reset();
            }));
      


        indexerButton.onTrue(indexer.run(() -> indexer.setIdle()));
        indexerButton.onTrue(agitator.run(() -> agitator.setIdle(45)));
        intakeButton.onTrue(intake.run(() -> intake.on(-300)));
        intakeEctButton.onTrue(intake.run(() -> intake.on(100)));
        indexerButton.onFalse(indexer.run(() -> indexer.off()));
        indexerJam.onTrue(indexer.run(() -> indexer.jam()));
        indexerJam.onFalse(indexer.run(() -> indexer.off()));
        indexerButton.onFalse(agitator.run(() -> agitator.setIdle(0)));


    }

    public Command getAutonomousCommand() {
    return Commands.none();
  }
}
