// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Joystick joystick = new Joystick(1);
    private final Joystick joystick2 = new Joystick(0);
    private final Trigger indexerButton = new JoystickButton(joystick, 1);
    private final Trigger intakeButton = new JoystickButton(joystick, 3);
    private final Trigger intakeEctButton = new JoystickButton(joystick, 4);
    private final Trigger indexerJam = new JoystickButton(joystick, 5);
    private final Trigger gyroResetButton = new JoystickButton(joystick, 8);

    private final Trigger flywheel1trButton = new JoystickButton(joystick2, 1);
    private final Trigger flywheel2trButton = new JoystickButton(joystick2, 2);
    private final Trigger flywheel3trButton = new JoystickButton(joystick2, 3);
    private final Trigger flywheel4trButton = new JoystickButton(joystick2, 4);
    private final Trigger flywheel5trButton = new JoystickButton(joystick2, 5);
    private final Trigger flywheel6trButton = new JoystickButton(joystick2, 6);
    private final Trigger flywheel7trButton = new JoystickButton(joystick2, 7);
    private final Trigger flywheel8trButton = new JoystickButton(joystick2, 8);
    private final Trigger flywheel9trButton = new JoystickButton(joystick2, 9);
    private final Trigger flywheel10trButton = new JoystickButton(joystick2, 10);
    private final Trigger flywheel11trButton = new JoystickButton(joystick2, 11);
    private final Trigger flywheel12trButton = new JoystickButton(joystick2, 12);
    private final Trigger flywheel13trButton = new JoystickButton(joystick2, 13);
    private final Trigger flywheel14trButton = new JoystickButton(joystick2, 14);
    private final Trigger flywheel15trButton = new JoystickButton(joystick2, 15);
    private final Trigger OFFButton = new JoystickButton(joystick2, 20);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(drive);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        pigeon2 = new Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus);

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
    // Drive train logic ------------------------------------------------------------------------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getY() / 2 * MaxSpeed)
                    .withVelocityY(-joystick.getX() / 2 * MaxSpeed)
                    .withRotationalRate((-joystick.getZ()))
            )
        );
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        drivetrain.registerTelemetry(logger::telemeterize);
    // ------------------------------------------------------------------------------

    
        gyroResetButton.onTrue(Commands.runOnce(() -> pigeon2.reset()));

        flywheel1trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(3200)));
        flywheel2trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(3400)));
        flywheel3trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(3600)));
        flywheel4trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(3800)));
        flywheel5trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(4000)));
        flywheel6trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(4200)));
        flywheel7trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(4400)));
        flywheel8trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(4600)));
        flywheel9trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(4800)));
        flywheel10trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(5000)));
        flywheel11trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(5200)));
        flywheel12trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(5400)));
        flywheel13trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(5600)));
        flywheel14trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(5800)));
        flywheel15trButton.toggleOnTrue(flywheel.run(() -> flywheel.spinAtRpm(5950)));

        indexerButton.whileTrue(
            indexer.idleCommand()
                .alongWith(agitator.idleCommand())
        );

        indexerJam.whileTrue(indexer.jamCommand());

        intakeButton.whileTrue(intake.intakeCommand());

        intakeEctButton.whileTrue(intake.ejectCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}