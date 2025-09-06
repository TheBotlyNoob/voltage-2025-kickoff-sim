package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class SimContainer {

  @Getter protected final SwerveDriveSimulation driveSim;

  private static final DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofPigeon2())
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(
              COTS.ofMark4(
                  DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                  DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                  COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                  3)) // L3 Gear ratio
          // Configures the track length and track width (spacing between swerve modules)
          .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
          // Configures the bumper size (dimensions of the robot bumper)
          .withBumperSize(Inches.of(30), Inches.of(30));

  protected final SimulatedArena arena;

  public SimContainer() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      throw new IllegalStateException("SimContainer can only be instantiated in SIM mode");
    }

    arena = new BlankSimArena();

    driveSim =
        new SwerveDriveSimulation(driveTrainSimulationConfig, new Pose2d(1, 1, Rotation2d.kZero));
    arena.addDriveTrainSimulation(driveSim);
  }

  public void simulationInit(ResetOdo resetOdometry) {
    arena.resetFieldForAuto();
    resetOdometry.apply(driveSim.getSimulatedDriveTrainPose());
  }

  public void simulationPeriodic() {
    arena.simulationPeriodic();

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  @FunctionalInterface
  public interface ResetOdo {

    void apply(Pose2d pose);
  }
}
