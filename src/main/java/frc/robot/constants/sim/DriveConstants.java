package frc.robot.constants.sim;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIOSim;

// ideally all of these constants are the same as their real counterparts,
// but they can be changed if needed
public class DriveConstants implements Drive.Constants, Module.Constants, ModuleIOSim.Constants {

  @Override
  public int driveCurrentLimit() {
    return Constants.current.drive.driveCurrentLimit();
  }

  @Override
  public int turnCurrentLimit() {
    return Constants.current.drive.turnCurrentLimit();
  }

  @Override
  public double driveKp() {
    return Constants.current.drive.driveKp();
  }

  @Override
  public double driveKd() {
    return Constants.current.drive.driveKd();
  }

  @Override
  public double driveKs() {
    return Constants.current.drive.driveKs();
  }

  @Override
  public double driveKv() {
    return Constants.current.drive.driveKv();
  }

  @Override
  public double turnKp() {
    return Constants.current.drive.turnKp();
  }

  @Override
  public double turnKd() {
    return Constants.current.drive.turnKd();
  }

  @Override
  public Translation2d[] moduleTranslations() {
    return Constants.current.drive.moduleTranslations();
  }

  @Override
  public double driveBaseRadius() {
    return Constants.current.drive.driveBaseRadius();
  }

  @Override
  public double maxSpeed() {
    return Constants.current.drive.maxSpeed();
  }

  @Override
  public PIDConstants translationPID() {
    return Constants.current.drive.translationPID();
  }

  @Override
  public PIDConstants rotationPID() {
    return Constants.current.drive.rotationPID();
  }

  @Override
  public RobotConfig ppRobotConfig() {
    return Constants.current.drive.ppRobotConfig();
  }

  @Override
  public double odometryFrequency() {
    return 100.0;
  }

  @Override
  public double wheelRadiusMeters() {
    return Constants.current.drive.wheelRadiusMeters();
  }
}
