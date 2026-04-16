package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTable {

  private static double fudgeFactor = 0.11;

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
    table.put(0.000 - fudgeFactor, new ShotSetpoint(3000));
    table.put(1.34 - fudgeFactor, new ShotSetpoint(3000));
    table.put(1.75 - fudgeFactor, new ShotSetpoint(3050));
    table.put(2.46 - fudgeFactor, new ShotSetpoint(3400));
    table.put(2.89 - fudgeFactor, new ShotSetpoint(4000));
    table.put(3.1496 - fudgeFactor, new ShotSetpoint(4375));
    table.put(3.3528 - fudgeFactor, new ShotSetpoint(5200));
    table.put(3.5 - fudgeFactor, new ShotSetpoint(5800));
    table.put(3.6 - fudgeFactor, new ShotSetpoint(5900));
  }

  public static ShotSetpoint get(double distanceMeters) {
    return table.get(distanceMeters).trimSetpoint();
  }

}