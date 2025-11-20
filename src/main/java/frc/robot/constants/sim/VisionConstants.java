package frc.robot.constants.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision.CameraConfig;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class VisionConstants implements Vision.Constants, VisionIOPhotonVisionSim.Constants {

  @Override
  public AprilTagFieldLayout aprilTagLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  @Override
  public double maxAmbiguity() {
    return Constants.current.vision.maxAmbiguity();
  }

  @Override
  public double maxZError() {
    return Constants.current.vision.maxZError();
  }

  @Override
  public double linearStdDevBaseline() {
    return Constants.current.vision.linearStdDevBaseline();
  }

  @Override
  public double angularStdDevBaseline() {
    return Constants.current.vision.angularStdDevBaseline();
  }

  @Override
  public double linearStdDevMegatag2Factor() {
    return Constants.current.vision.linearStdDevMegatag2Factor();
  }

  @Override
  public double angularStdDevMegatag2Factor() {
    return Constants.current.vision.angularStdDevMegatag2Factor();
  }

  @Override
  public double[] cameraStdDevFactors() {
    return new double[0];
  }

  public CameraConfig[] cameraConfigs() {
    return Constants.current.vision.cameraConfigs();
  }
}
