package frc.robot.commands;

import static frc.robot.constants.JrConstants.DriveConstants.rotationPID;
import static frc.robot.constants.JrConstants.DriveConstants.translationPID;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends Command {

  private final Drive dt;
  private final PathPlannerTrajectoryState goalState;

  private final double tolerance = 0.1;

  private final PPHolonomicDriveController pid =
      new PPHolonomicDriveController(translationPID, rotationPID);

  /**
   * Command to drive to a specific pose. Without a specified target speed, the command will end
   * with a stop at the target pose.
   * <p>
   * Does not use pathfinding; just drives in a straight line to the target. In the future, could be
   * improved with obstacle avoidance if we need longer distances.
   *
   * @param drivetrain The drive subsystem to use.
   * @param goalPose The target pose to drive to.
   */
  public DriveToPose(Drive drivetrain, Pose2d goalPose) {
    dt = drivetrain;
    goalState = new PathPlannerTrajectoryState();
    goalState.pose = goalPose;

    addRequirements(dt);
  }

  /**
   * Command to drive to a specific pose.
   *
   * @param drivetrain The drive subsystem to use.
   * @param goalState The target state to drive to, including pose and speed.
   */
  public DriveToPose(Drive drivetrain, PathPlannerTrajectoryState goalState) {
    dt = drivetrain;
    this.goalState = goalState;

    addRequirements(dt);
  }

  @Override
  public void execute() {
    dt.runVelocity(pid.calculateRobotRelativeSpeeds(dt.getPose(), goalState));
  }

  // TODO: figure out why it never finishes.
  @Override
  public boolean isFinished() {
    boolean translationDone =
        Math.abs(dt.getPose().getTranslation().getDistance(goalState.pose.getTranslation()))
            < tolerance;

    boolean rotationDone =
        MathUtil.isNear(
            dt.getRotation().getDegrees(), goalState.pose.getRotation().getDegrees(), tolerance);

    ChassisSpeeds speedDiff = dt.getChassisSpeeds().minus(goalState.fieldSpeeds);

    boolean speedDone =
        speedDiff.vxMetersPerSecond < tolerance
            && speedDiff.vyMetersPerSecond < tolerance
            && speedDiff.omegaRadiansPerSecond < tolerance;

    return translationDone && rotationDone && speedDone;
  }
}
