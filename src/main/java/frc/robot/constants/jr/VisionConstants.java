package frc.robot.constants.jr;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVision.CameraConfig;

public class VisionConstants implements Vision.Constants, VisionIOPhotonVision.Constants {

  @Override
  public AprilTagFieldLayout aprilTagLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  @Override
  public double maxAmbiguity() {
    return 0.3;
  }

  @Override
  public double maxZError() {
    return 0.75;
  }

  public CameraConfig[] cameraConfigs() {
    CameraConfig camera0 = new CameraConfig();
    camera0.cameraName = "camera_0";
    camera0.robotToCamera = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));

    return new CameraConfig[] {camera0};
  }

  @Override
  public double[] cameraStdDevFactors() {
    return new double[] {1.0};
  }

  @Override
  public double linearStdDevBaseline() {
    // Meters
    return 0.02;
  }

  @Override
  public double angularStdDevBaseline() {
    return 0.06;
  }

  @Override
  public double linearStdDevMegatag2Factor() {
    return 0.5;
  }

  @Override
  public double angularStdDevMegatag2Factor() {
    return Double.POSITIVE_INFINITY;
  }
}
