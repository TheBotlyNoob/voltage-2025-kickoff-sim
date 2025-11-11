package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {

    public boolean motorConnected = false;

    public double positionRad = 0.0;

    public double velocityRadPerSec = 0.0;

    public double leadAppliedVolts = 0.0;
    public double leadCurrentAmps = 0.0;

    public double followAppliedVolts = 0.0;
    public double followCurrentAmps = 0.0;

    public boolean atTop = false;
    public boolean atBottom = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Set the target height of the elevator in meters.
   *
   * @param rads The target motor position in radians.
   */
  default void setTargetPosition(double rads) {}

  /** Set the motor to a set open loop. Overrides target position. */
  default void setVoltage(double volts) {}

  /** Overrides the lead encoder position to a known value. */
  default void setEncoderPosition(double rads) {}
}
