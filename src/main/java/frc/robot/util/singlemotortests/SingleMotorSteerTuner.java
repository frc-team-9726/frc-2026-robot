package frc.robot.util.singlemotortests;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * TunerConstants starting points (copy these in first):
 *   kP = 10   kI = 0   kD = 0.00001   kS = 0.1   kV = 0.050   kA = 0
 *
 * Tuning procedure:
 *   1. Disconnect the swerve module from the robot frame so the wheel can
 *      spin freely (or at least so you can observe it safely).
 *   2. Set Enabled = 1 and command SetpointDegrees = 0 to confirm the module
 *      homes correctly.
 *   3. Command ±90° and ±180° steps while watching the response in
 *      AdvantageScope or SmartDashboard.
 *   4. Raise kP until the module snaps to position quickly without oscillating.
 *   5. Add kD to damp any oscillation; it is usually small relative to kP.
 *   6. kS compensates for stiction — increase it if the module stalls just
 *      before reaching the setpoint.
 *   7. Enable MotionMagic (UseMotionMagic = 1) and tune MaxVelRPS /
 *      MaxAccelRPSperSec so the module sweeps smoothly without slamming.
 *   8. Copy final gains back into TunerConstants.steerGains.
 */
public class SingleMotorSteerTuner extends SubsystemBase {

    private static final int     kMotorCanId    = 2;
    private static final boolean kInverted      = false;
    private static final double  kSteerGearRatio = 12.1;

    private final TalonFX motor = new TalonFX(kMotorCanId);

    private final PositionVoltage    positionRequest =
        new PositionVoltage(0.0).withSlot(0).withEnableFOC(true);
    private final MotionMagicVoltage mmPositionRequest =
        new MotionMagicVoltage(0.0).withSlot(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final LoggedTunableNumber enabled =
        new LoggedTunableNumber("SteerTuner/Enabled", 0.0);

    private final LoggedTunableNumber setpointDegrees =
        new LoggedTunableNumber("SteerTuner/SetpointDegrees", 0.0);

    private final LoggedTunableNumber useMotionMagic =
        new LoggedTunableNumber("SteerTuner/UseMotionMagic", 0.0);

    private final LoggedTunableNumber maxVelRps =
        new LoggedTunableNumber("SteerTuner/MaxVelRPS", 6.05);

    private final LoggedTunableNumber maxAccelRps =
        new LoggedTunableNumber("SteerTuner/MaxAccelRPSperSec", 12.1);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("SteerTuner/kP", 10.0);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("SteerTuner/kI", 0.0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("SteerTuner/kD", 0.00001);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("SteerTuner/kS", 0.1);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("SteerTuner/kV", 0.050);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("SteerTuner/kA", 0.0);

    private final LoggedTunableNumber minOut =
        new LoggedTunableNumber("SteerTuner/MinOut", -1.0);
    private final LoggedTunableNumber maxOut =
        new LoggedTunableNumber("SteerTuner/MaxOut",  1.0);

    private final int changeId = System.identityHashCode(this);

    public SingleMotorSteerTuner() {
        applyConfig();
    }

    private void applyConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs motorOut = cfg.MotorOutput;
        motorOut.Inverted = kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        motorOut.NeutralMode          = NeutralModeValue.Brake;
        motorOut.PeakForwardDutyCycle = maxOut.get();
        motorOut.PeakReverseDutyCycle = minOut.get();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = kP.get();
        slot0.kI = kI.get();
        slot0.kD = kD.get();
        slot0.kS = kS.get();
        slot0.kV = kV.get();
        slot0.kA = kA.get();

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = maxVelRps.get();
        mm.MotionMagicAcceleration   = maxAccelRps.get();

        cfg.CurrentLimits.StatorCurrentLimit       = 60;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(cfg);
    }

    private double degreesToMotorRotations(double degrees) {
        return (degrees / 360.0) * kSteerGearRatio;
    }

    private double motorRotationsToDegrees(double rotations) {
        return (rotations / kSteerGearRatio) * 360.0;
    }

    @Override
    public void periodic() {
        LoggedTunableNumber.ifChanged(
            changeId,
            this::applyConfig,
            kP, kI, kD, kS, kV, kA, minOut, maxOut, maxVelRps, maxAccelRps);

        double targetMotorRot = degreesToMotorRotations(setpointDegrees.get());

        if (enabled.get() > 0.5) {
            if (useMotionMagic.get() > 0.5) {
                motor.setControl(mmPositionRequest.withPosition(targetMotorRot));
            } else {
                motor.setControl(positionRequest.withPosition(targetMotorRot));
            }
        } else {
            motor.setControl(neutralRequest);
        }

        // Telemetry
        double measuredMotorRot  = motor.getPosition().getValueAsDouble();
        double measuredDegrees   = motorRotationsToDegrees(measuredMotorRot);
        double errorDegrees      = setpointDegrees.get() - measuredDegrees;
        double motorVelRps       = motor.getVelocity().getValueAsDouble();
        double appliedOutput     = motor.getDutyCycle().getValueAsDouble();
        double statorCurrent     = motor.getStatorCurrent().getValueAsDouble();

        SmartDashboard.putNumber("SteerTuner/SetpointDegrees",  setpointDegrees.get());
        SmartDashboard.putNumber("SteerTuner/MeasuredDegrees",  measuredDegrees);
        SmartDashboard.putNumber("SteerTuner/ErrorDegrees",     errorDegrees);
        SmartDashboard.putNumber("SteerTuner/MotorVelRPS",      motorVelRps);
        SmartDashboard.putNumber("SteerTuner/AppliedOutput",    appliedOutput);
        SmartDashboard.putNumber("SteerTuner/StatorCurrentA",   statorCurrent);
        SmartDashboard.putNumber("SteerTuner/TargetMotorRot",   targetMotorRot);
        SmartDashboard.putNumber("SteerTuner/MeasuredMotorRot", measuredMotorRot);

        Logger.recordOutput("SteerTuner/SetpointDegrees",  setpointDegrees.get());
        Logger.recordOutput("SteerTuner/MeasuredDegrees",  measuredDegrees);
        Logger.recordOutput("SteerTuner/ErrorDegrees",     errorDegrees);
        Logger.recordOutput("SteerTuner/MotorVelRPS",      motorVelRps);
        Logger.recordOutput("SteerTuner/StatorCurrentA",   statorCurrent);
    }
}