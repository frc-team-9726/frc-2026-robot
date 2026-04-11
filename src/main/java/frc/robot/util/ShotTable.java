package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTable {

  public record ShotSetpoint(double shooterSpeed) {
    public ShotSetpoint trimSetpoint() {
      return new ShotSetpoint(shooterSpeed);
    }
  }

  private static final Interpolator<ShotSetpoint> shotInterpolator =
      (a, b, t) ->
          new ShotSetpoint(
              MathUtil.interpolate(a.shooterSpeed(), b.shooterSpeed(), t));

  private static final InterpolatingTreeMap<Double, ShotSetpoint> table =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), shotInterpolator);

  static {
    table.put(0.000, new ShotSetpoint(3200));
    table.put(0.275, new ShotSetpoint(3400));
    table.put(0.550, new ShotSetpoint(3600));
    table.put(0.825, new ShotSetpoint(3800));
    table.put(1.100, new ShotSetpoint(4000));
    table.put(1.375, new ShotSetpoint(4200));
    table.put(1.650, new ShotSetpoint(4400));
    table.put(1.925, new ShotSetpoint(4600));
    table.put(2.200, new ShotSetpoint(4800));
    table.put(2.475, new ShotSetpoint(5000));
    table.put(2.750, new ShotSetpoint(5200));
    table.put(3.025, new ShotSetpoint(5400));
    table.put(3.300, new ShotSetpoint(5600));
    table.put(3.575, new ShotSetpoint(5800));
    table.put(3.850, new ShotSetpoint(5950));
  }

  public static ShotSetpoint get(double distanceMeters) {
    return table.get(distanceMeters).trimSetpoint();
  }

}