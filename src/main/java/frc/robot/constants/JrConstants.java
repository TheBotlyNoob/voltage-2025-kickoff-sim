package frc.robot.constants;

import edu.wpi.first.math.util.Units;

// constants for each subsystem
public class JrConstants {

  public static class ElevatorConstants {

    public static class PID {

      public static final double kP = 0.0; // not tuned
      public static final double kI = 0.0; // not tuned
      public static final double kD = 0.0; // not tuned
    }

    public static class FF { // not tuned

      public static final double kS = 0.1; // static friction (V)
      public static final double kG = 0.65; // gravity (V)
      public static final double kV = 1.6; // volts per velocity (V/(m/s))
      public static final double kA = 0.0; // volts per acceleration (V/(m/s^2))
    }

    public static double kCarriageMass = 10.0; // kg
    public static double kElevatorDrumRadius = Units.inchesToMeters(1.0); // meters
    public static double kElevatorGearing =
        10.0; // 10:1 reduction (10 motor rotations -> 1 drum rotation)
    public static double kMinElevatorHeightMeters = 0.0; // meters
    public static double kMaxElevatorHeightMeters = 30.0; // meters

    public static double kManualVoltage = 6.0;

    public static int kElevatorLeaderCanId = 10;
    public static int kElevatorFollowerCanId = 11;

    public static final double kMaxVelocity = 3.5;
    public static final double kMaxAcceleration = 3.0;
    public static final double kDecelProp = 0.5;

    public static final int kBottomLimitSwitch = 4;
    public static final int kTopLimitSwitch = 8;
  }
}
