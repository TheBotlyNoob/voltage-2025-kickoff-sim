package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.SimulatedArena;

public class BlankSimArena extends SimulatedArena {

  public static class BlankSimArenaObstacles extends SimulatedArena.FieldMap {

    public BlankSimArenaObstacles() {
      super();

      // blue wall
      super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

      // blue coral stations
      super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(1.672, 0));
      super.addBorderLine(new Translation2d(0, 6.782), new Translation2d(1.672, 8.052));

      // red wall
      super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

      // red coral stations
      super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548 - 1.672, 0));
      super.addBorderLine(
          new Translation2d(17.548, 6.782), new Translation2d(17.548 - 1.672, 8.052));

      // upper walls
      super.addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(11, 8.052));
      super.addBorderLine(new Translation2d(12, 8.052), new Translation2d(17.548 - 1.672, 8.052));

      // lower walls
      super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(5.8, 0));
      super.addBorderLine(new Translation2d(6.3, 0), new Translation2d(17.548 - 1.672, 0));
    }
  }

  public BlankSimArena() {
    super(new BlankSimArenaObstacles());
  }

  @Override
  public void placeGamePiecesOnField() {}
}
