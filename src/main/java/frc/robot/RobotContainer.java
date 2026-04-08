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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.util.singlemotortests.SingleMotorVelocityPIDFKrakenTest;

public class RobotContainer {
    private final SingleMotorVelocityPIDFKrakenTest singleMotorVelocityPIDFKrakenTest;

    private final Pigeon2 pigeon2;
    private final Limelight limelight = new Limelight("");
    // private final Flywheel flywheel = new Flywheel(14);
    private final Indexer indexer = new Indexer(20);
    private final Agitator agitator = new Agitator(4);
    private final Intake intake = new Intake(2);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Joystick keyboard = new Joystick(0);
    private final Joystick joystick1 = new Joystick(1);

    private final JoystickButton gyroResetButton = new JoystickButton(joystick1, 8);

    private final JoystickButton JOYSTICK_BUTTON_1 = new JoystickButton(joystick1, 1);
    private final JoystickButton JOYSTICK_BUTTON_3 = new JoystickButton(joystick1, 3);
    private final JoystickButton JOYSTICK_BUTTON_4 = new JoystickButton(joystick1, 4);

    private final JoystickButton flywheel1trButton = new JoystickButton(keyboard, 1);
    private final JoystickButton flywheel2trButton = new JoystickButton(keyboard, 2);
    private final JoystickButton flywheel3trButton = new JoystickButton(keyboard, 3);
    private final JoystickButton flywheel4trButton = new JoystickButton(keyboard, 4);
    private final JoystickButton flywheel5trButton = new JoystickButton(keyboard, 5);
    private final JoystickButton flywheel6trButton = new JoystickButton(keyboard, 6);
    private final JoystickButton flywheel7trButton = new JoystickButton(keyboard, 7);
    private final JoystickButton flywheel8trButton = new JoystickButton(keyboard, 8);
    private final JoystickButton flywheel9trButton = new JoystickButton(keyboard, 9);
    private final JoystickButton flywheel10trButton = new JoystickButton(keyboard, 10);
    private final JoystickButton flywheel11trButton = new JoystickButton(keyboard, 11);
    private final JoystickButton flywheel12trButton = new JoystickButton(keyboard, 12);
    private final JoystickButton flywheel13trButton = new JoystickButton(keyboard, 13);
    private final JoystickButton flywheel14trButton = new JoystickButton(keyboard, 14);
    private final JoystickButton flywheel15trButton = new JoystickButton(keyboard, 15);
    private final JoystickButton OFFButton = new JoystickButton(keyboard, 20);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(drive);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        pigeon2 = new Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus);

        singleMotorVelocityPIDFKrakenTest = new SingleMotorVelocityPIDFKrakenTest();
        
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick1.getY() / 2 * MaxSpeed)
                    .withVelocityY(-joystick1.getX() / 2 * MaxSpeed)
                    .withRotationalRate((-joystick1.getZ()))
            )
        );

        // final var idle = new SwerveRequest.Idle();
        // RobotModeJoystickButtons.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        drivetrain.registerTelemetry(logger::telemeterize);

        // flywheel1trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3200)));
        // flywheel2trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3400)));
        // flywheel3trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3600)));
        // flywheel4trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3800)));
        // flywheel5trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4000)));
        // flywheel6trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4200)));
        // flywheel7trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4400)));
        // flywheel8trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4600)));
        // flywheel9trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4800)));
        // flywheel10trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5000)));
        // flywheel11trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5200)));
        // flywheel12trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5400)));
        // flywheel13trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5600)));
        // flywheel14trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5800)));
        // flywheel15trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5950)));

        gyroResetButton.onTrue(Commands.runOnce(() -> pigeon2.reset()));


        JOYSTICK_BUTTON_1.onTrue(Commands.runOnce(() -> {
            indexer.intake();
            agitator.intake();
        }));

        JOYSTICK_BUTTON_3.onTrue(Commands.runOnce(() -> {
            agitator.intake();
            intake.intake();
        }));

        JOYSTICK_BUTTON_4.onTrue(Commands.runOnce(() -> {
            agitator.outake();
            intake.outake();
        }));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}