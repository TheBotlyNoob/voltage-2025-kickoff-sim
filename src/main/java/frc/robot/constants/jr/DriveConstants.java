package frc.robot.constants.jr;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;

public class DriveConstants
    implements Drive.Constants,
        Module.Constants,
        GyroIOPigeon2.Constants,
        ModuleIOSparkFlex.Constants {

  @Override
  public int pigeonCanId() {
    return 13;
  }

  @Override
  public Rotation2d[] zeroRotations() {
    return new Rotation2d[] {
      new Rotation2d(0), new Rotation2d(0), new Rotation2d(0), new Rotation2d(0)
    };
  }

  @Override
  public int[] driveCanIds() {
    return new int[] {6, 7, 52, 2};
  }

  @Override
  public int[] turnCanIds() {
    return new int[] {5, 4, 8, 3};
  }

  @Override
  public int driveCurrentLimit() {
    return 60;
  }

  @Override
  public int turnCurrentLimit() {
    return 20;
  }

  @Override
  public double driveKp() {
    return .5;
  }

  @Override
  public double driveKd() {
    return 0.0;
  }

  @Override
  public double driveKs() {
    return 0.0;
  }

  @Override
  public double driveKv() {
    return 0.3;
  }

  public static final double wheelRadiusMeters = Units.inchesToMeters(4);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Rotor Rotations -> Wheel Radians
  @Override
  public double driveEncoderPositionFactor() {
    return 2 * Math.PI / driveMotorReduction;
  }

  // Rotor RPM -> Wheel Rad/Sec
  @Override
  public double driveEncoderVelocityFactor() {
    return (2 * Math.PI) / 60.0 / driveMotorReduction;
  }

  @Override
  public double turnKp() {
    return 1.5;
  }

  @Override
  public double turnKd() {
    return 0;
  }

  @Override
  public double turnPIDMinInput() {
    // Radians
    return 0;
  }

  @Override
  public double turnPIDMaxInput() {
    // Radians
    return 2 * Math.PI;
  }

  @Override
  public double turnEncoderPositionFactor() {
    // Rotations -> Radians
    return 2 * Math.PI;
  }

  @Override
  public double turnEncoderVelocityFactor() {
    // RPM -> Rad/Sec
    return (2 * Math.PI) / 60.0;
  }

  @Override
  public boolean turnInverted() {
    return false;
  }

  @Override
  public boolean turnEncoderInverted() {
    return true;
  }

  public static final double trackWidth = Units.inchesToMeters(24.75);
  public static final double wheelBase = Units.inchesToMeters(24.75);

  @Override
  public Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };
  }

  @Override
  public double driveBaseRadius() {
    return Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  }

  @Override
  public PIDConstants translationPID() {
    return new PIDConstants(5.0, 0.0, 0.0);
  }

  @Override
  public PIDConstants rotationPID() {
    return new PIDConstants(5.0, 0.0, 0.0);
  }

  public static final double robotMassKg = 24.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;

  @Override
  public RobotConfig ppRobotConfig() {
    return new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
            wheelRadiusMeters,
            maxSpeed(),
            wheelCOF,
            driveGearbox.withReduction(driveMotorReduction),
            driveCurrentLimit(),
            1),
        moduleTranslations());
  }

  // Meters/Sec
  public double maxSpeed() {
    return 4.0;
  }

  @Override
  public double odometryFrequency() {
    return 100.0;
  }

  @Override
  public double wheelRadiusMeters() {
    return Units.inchesToMeters(4);
  }
}
