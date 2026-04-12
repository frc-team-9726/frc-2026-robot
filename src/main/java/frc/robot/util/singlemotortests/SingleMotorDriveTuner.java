package frc.robot.util.singlemotortests;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * TunerConstants starting points (copy these in first):
 *   kP = 0.005   kI = 0   kD = 0.1   kS = 0   kV = 0.0124
 *
 * Tuning procedure:
 *   1. Set kP, kI, kD, kS, kV to your TunerConstants values.
 *   2. Enable the motor (Enabled = 1) and command a moderate setpoint (~1500 RPM).
 *   3. Raise kV until measured RPM tracks the setpoint with minimal steady-state error.
 *   4. Raise kP to tighten dynamic tracking; reduce if oscillation appears.
 *   5. Add kD only if there is overshoot you cannot remove by lowering kP.
 *   6. Optionally enable MotionMagic (UseMotionMagicVelocity = 1) and tune MaxAccelRPSperSec
 *      for a smooth ramp — useful if wheel slip is an issue during acceleration.
 *   7. Copy final gains back into TunerConstants.driveGains.
 */
public class SingleMotorDriveTuner extends SubsystemBase {

    private static final int     kMotorCanId = 7;
    private static final boolean kInverted   = false;

    private final TalonFX motor = new TalonFX(kMotorCanId);

    private final VelocityVoltage            velocityRequest =
        new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage mmVelocityRequest =
        new MotionMagicVelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final LoggedTunableNumber enabled =
        new LoggedTunableNumber("DriveTuner/Enabled", 0.0);
    private final LoggedTunableNumber setpointRpm =
        new LoggedTunableNumber("DriveTuner/SetpointRPM", 0.0);
    private final LoggedTunableNumber useMotionMagicVel =
        new LoggedTunableNumber("DriveTuner/UseMotionMagicVelocity", 0.0);

    private final LoggedTunableNumber maxAccelRps =
        new LoggedTunableNumber("DriveTuner/MaxAccelRPSperSec", 6.67);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("DriveTuner/kP", 0.005);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("DriveTuner/kI", 0.0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("DriveTuner/kD", 0.1);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("DriveTuner/kS", 0.0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("DriveTuner/kV", 0.0124);

    private final LoggedTunableNumber kA = new LoggedTunableNumber("DriveTuner/kA", 0.0);

    private final LoggedTunableNumber minOut =
        new LoggedTunableNumber("DriveTuner/MinOut", -1.0);
    private final LoggedTunableNumber maxOut =
        new LoggedTunableNumber("DriveTuner/MaxOut",  1.0);

    private final int changeId = System.identityHashCode(this);

    public SingleMotorDriveTuner() {
        applyConfig();
    }

    private void applyConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs motorOut = cfg.MotorOutput;
        motorOut.Inverted   = kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        motorOut.NeutralMode            = NeutralModeValue.Coast;
        motorOut.PeakForwardDutyCycle   = maxOut.get();
        motorOut.PeakReverseDutyCycle   = minOut.get();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = kP.get();
        slot0.kI = kI.get();
        slot0.kD = kD.get();
        slot0.kS = kS.get();
        slot0.kV = kV.get();
        slot0.kA = kA.get();

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicAcceleration = maxAccelRps.get();

        cfg.CurrentLimits.SupplyCurrentLimit       = 60;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        LoggedTunableNumber.ifChanged(
            changeId,
            this::applyConfig,
            kP, kI, kD, kS, kV, kA, minOut, maxOut, maxAccelRps);

        if (enabled.get() > 0.5) {
            double setpointRps = setpointRpm.get() / 60.0;

            if (useMotionMagicVel.get() > 0.5) {
                motor.setControl(mmVelocityRequest.withVelocity(setpointRps));
            } else {
                motor.setControl(velocityRequest.withVelocity(setpointRps));
            }
        } else {
            motor.setControl(neutralRequest);
        }

        double measuredRpm    = motor.getVelocity().getValueAsDouble() * 60.0;
        double appliedOutput  = motor.getDutyCycle().getValueAsDouble();
        double busVoltage     = motor.getSupplyVoltage().getValueAsDouble();
        double statorCurrent  = motor.getStatorCurrent().getValueAsDouble();

        SmartDashboard.putNumber("DriveTuner/MeasuredRPM",    measuredRpm);
        SmartDashboard.putNumber("DriveTuner/SetpointRPM",    setpointRpm.get());
        SmartDashboard.putNumber("DriveTuner/AppliedOutput",  appliedOutput);
        SmartDashboard.putNumber("DriveTuner/BusVoltage",     busVoltage);
        SmartDashboard.putNumber("DriveTuner/StatorCurrentA", statorCurrent);
        SmartDashboard.putNumber("DriveTuner/ErrorRPM",       setpointRpm.get() - measuredRpm);

        Logger.recordOutput("DriveTuner/MeasuredRPM",    measuredRpm);
        Logger.recordOutput("DriveTuner/SetpointRPM",    setpointRpm.get());
        Logger.recordOutput("DriveTuner/ErrorRPM",       setpointRpm.get() - measuredRpm);
        Logger.recordOutput("DriveTuner/StatorCurrentA", statorCurrent);
    }
}