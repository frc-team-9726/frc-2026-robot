// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AdaptiveHubAiming;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
    // FIX 1: Removed duplicate Pigeon2 instance — the drivetrain already owns one
    // internally via TunerConstants. Creating a second one at the same CAN ID
    // caused gyro resets to be invisible to the pose estimator.

    private final Flywheel flywheel = new Flywheel(14);
    private final Indexer indexer = new Indexer(34);
    private final Agitator agitator = new Agitator(4);
    private final Intake intake = new Intake(2);
    private final Limelight limelight = new Limelight("limelight");

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);

    // 2 = 0.5 seconds to max speed, 4 = 0.25 seconds etc...
    private final SlewRateLimiter xLimiter   = new SlewRateLimiter(2.0);
    private final SlewRateLimiter yLimiter   = new SlewRateLimiter(2.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveWithAngle =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(3.2, 0.0, 0.0);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Joystick keyboard  = new Joystick(0);
    private final Joystick joystick1 = new Joystick(1);
    private final Joystick joystick2 = new Joystick(2);

    private final JoystickButton gyroResetButton = new JoystickButton(joystick1, 8);

    private final JoystickButton JOYSTICK_BUTTON_1 = new JoystickButton(joystick1, 1);
    private final JoystickButton JOYSTICK_BUTTON_2 = new JoystickButton(joystick1, 2);
    private final JoystickButton JOYSTICK_BUTTON_3 = new JoystickButton(joystick1, 3);
    private final JoystickButton JOYSTICK_BUTTON_4 = new JoystickButton(joystick1, 4);
    private final JoystickButton JOYSTICK_BUTTON_13 = new JoystickButton(joystick1, 13);
    private final JoystickButton JOYSTICK_BUTTON_14 = new JoystickButton(joystick1, 14);
    private final JoystickButton JOYSTICK_BUTTON_15 = new JoystickButton(joystick1, 15);
    private final JoystickButton JOYSTICK_BUTTON_16 = new JoystickButton(joystick1, 16);

    private final JoystickButton flywheel1trButton  = new JoystickButton(keyboard, 1);
    private final JoystickButton flywheel2trButton  = new JoystickButton(keyboard, 2);
    private final JoystickButton flywheel3trButton  = new JoystickButton(keyboard, 3);
    private final JoystickButton flywheel4trButton  = new JoystickButton(keyboard, 4);
    private final JoystickButton flywheel5trButton  = new JoystickButton(keyboard, 5);
    private final JoystickButton flywheel6trButton  = new JoystickButton(keyboard, 6);
    private final JoystickButton flywheel7trButton  = new JoystickButton(keyboard, 7);
    private final JoystickButton flywheel8trButton  = new JoystickButton(keyboard, 8);
    private final JoystickButton flywheel9trButton  = new JoystickButton(keyboard, 9);
    private final JoystickButton flywheel10trButton = new JoystickButton(keyboard, 10);
    private final JoystickButton flywheel11trButton = new JoystickButton(keyboard, 11);
    private final JoystickButton flywheel12trButton = new JoystickButton(keyboard, 12);
    private final JoystickButton flywheel13trButton = new JoystickButton(keyboard, 13);
    private final JoystickButton flywheel14trButton = new JoystickButton(keyboard, 14);
    private final JoystickButton flywheel15trButton = new JoystickButton(keyboard, 15);
    private final JoystickButton flywheel16trButton = new JoystickButton(keyboard, 16);
    private final JoystickButton OFFButton          = new JoystickButton(keyboard, 20);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(drive, limelight);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("shoot speed close", Commands.runOnce(() -> {
            flywheel.setSetpointRpm(3400);
        }));

        NamedCommands.registerCommand("shoot", Commands.runOnce(() -> {
            indexer.intake();
            agitator.intake();
        }));

        NamedCommands.registerCommand("stop shoot", Commands.runOnce(() -> {
            indexer.stop();
            agitator.stop();
        }));

        NamedCommands.registerCommand("intake", Commands.runOnce(() -> {
            agitator.intake();
            intake.intake();
        }));

        NamedCommands.registerCommand("auto intake", Commands.runOnce(() -> {
            agitator.outake();
            intake.autoIntake();
        }));

        NamedCommands.registerCommand("auto outake", Commands.runOnce(() -> {
            agitator.outake();
            intake.autoOutake();
        }));

        NamedCommands.registerCommand("stop intake", Commands.runOnce(() -> {
            agitator.stop();
            intake.stop();
        }));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // --- Default drive: joystick1, normal rate-based turning ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double rawX   = -joystick1.getY() * 0.9;
                double rawY   = -joystick1.getX() * 0.9;
                double rawRot = -joystick1.getZ();

                double filteredX   = xLimiter.calculate(rawX);
                double filteredY   = yLimiter.calculate(rawY);
                double filteredRot = rotLimiter.calculate(rawRot);

                return drive
                    .withVelocityX(filteredX * MaxSpeed)
                    .withVelocityY(filteredY * MaxSpeed)
                    .withRotationalRate(filteredRot * MaxAngularRate * 0.7);
            })
        );

        new JoystickButton(joystick2, 1).whileTrue(
            drivetrain.applyRequest(() -> {
                double rawX = -joystick2.getY() * 0.9;
                double rawY = -joystick2.getX() * 0.9;
                double rotX = joystick2.getZ();

                double filteredX = xLimiter.calculate(rawX);
                double filteredY = yLimiter.calculate(rawY);

                if (Math.abs(rotX) > 0.2) {
                    return driveWithAngle
                        .withVelocityX(filteredX * MaxSpeed)
                        .withVelocityY(filteredY * MaxSpeed)
                        .withTargetDirection(Rotation2d.fromRadians(rotX * Math.PI));
                } else {
                    return drive
                        .withVelocityX(filteredX * MaxSpeed)
                        .withVelocityY(filteredY * MaxSpeed)
                        .withRotationalRate(0);
                }
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);
        
        AdaptiveHubAiming aimingCommand = new AdaptiveHubAiming(
            flywheel, drivetrain, limelight,
            DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red
        );

        AdaptiveHubAiming aimingCommandWithHeading = new AdaptiveHubAiming(
            flywheel, drivetrain, limelight,
            DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red
        );

        // JOYSTICK_BUTTON_2.whileTrue(aimingCommand);

        JOYSTICK_BUTTON_2.whileTrue(
            aimingCommandWithHeading.alongWith(
                drivetrain.runAimingInputs(
                    () -> xLimiter.calculate(-joystick1.getY() * 0.9) * MaxSpeed,
                    () -> yLimiter.calculate(-joystick1.getX() * 0.9) * MaxSpeed,
                    aimingCommandWithHeading::getFieldAimRotation
                )
            )
        );

        JOYSTICK_BUTTON_13.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(0)));

        flywheel1trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3200)));
        flywheel2trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3400)));
        flywheel3trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3600)));
        flywheel4trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3800)));
        flywheel5trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4000)));
        flywheel6trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4200)));
        flywheel7trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4400)));
        flywheel8trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4600)));
        flywheel9trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4800)));
        flywheel10trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5000)));
        flywheel11trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5200)));
        flywheel12trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5400)));
        flywheel13trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5600)));
        flywheel14trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5800)));
        flywheel15trButton.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5950)));

        JOYSTICK_BUTTON_14.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(5800)));
        JOYSTICK_BUTTON_15.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(4600)));
        JOYSTICK_BUTTON_16.onTrue(flywheel.run(() -> flywheel.setSetpointRpm(3400)));

        gyroResetButton.onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));



        JOYSTICK_BUTTON_1.onTrue(Commands.runOnce(() -> {
            indexer.intake();
            agitator.intake();
        }));
        JOYSTICK_BUTTON_1.onFalse(Commands.runOnce(() -> {
            indexer.stop();
            agitator.stop();
        }));

        JOYSTICK_BUTTON_3.onTrue(Commands.runOnce(() -> intake.intake()));
        JOYSTICK_BUTTON_3.onFalse(Commands.runOnce(() -> intake.stop()));

        JOYSTICK_BUTTON_4.onTrue(Commands.runOnce(() -> {
            agitator.outake();
            intake.outake();
        }));
        JOYSTICK_BUTTON_4.onFalse(Commands.runOnce(() -> {
            agitator.stop();
            intake.stop();
        }));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}