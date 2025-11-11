package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.SimConstants.ElevatorConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO, Simulatable {

  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor elevatorGearbox = DCMotor.getVex775Pro(4);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController positionPid =
      new ProfiledPIDController(
          ElevatorConstants.PID.kP,
          ElevatorConstants.PID.kI,
          ElevatorConstants.PID.kD,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward positionFF =
      new ElevatorFeedforward(
          ElevatorConstants.FF.kS,
          ElevatorConstants.FF.kG,
          ElevatorConstants.FF.kV,
          ElevatorConstants.FF.kA);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);

  private final Timer tickTimer = new Timer();

  private final MapleMotorSim motorSim =
      new MapleMotorSim(
          new SimMotorConfigs(
              elevatorGearbox,
              ElevatorConstants.kElevatorGearing,
              KilogramSquareMeters.of(0.004),
              Volts.of(0.5)));

  private final GenericMotorController motor =
      motorSim.useSimpleDCMotorController().withCurrentLimit(Amps.of(60));

  public ElevatorIOSim() {
    tickTimer.stop();
    tickTimer.reset();
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.positionRad = elevatorSim.getPositionMeters() / ElevatorConstants.kElevatorDrumRadius;
    inputs.velocityRadPerSec =
        elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.kElevatorDrumRadius;
    // inputs.appliedVolts = motorSim.getAppliedVoltage().in(Volts);
    // inputs.currentAmps = motorSim.getStatorCurrent().in(Amps);
  }

  public void setTargetPosition(double rads) {
    positionPid.setGoal(
        MathUtil.clamp(
            rads,
            ElevatorConstants.kMinElevatorHeightMeters,
            ElevatorConstants.kMaxElevatorHeightMeters));
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    if (positionPid.getGoal().position >= 0) {
      // With the setpoint value we run PID control like normal
      double pidOutput =
          positionPid.calculate(
              elevatorSim.getPositionMeters() / ElevatorConstants.kElevatorDrumRadius);
      double feedforwardOutput = positionFF.calculate(positionPid.getSetpoint().velocity);
      motor.requestVoltage(Volts.of(MathUtil.clamp(pidOutput + feedforwardOutput, -12, 12)));

      // In this method, we update our simulation of what our elevator is doing
      // First, we set our "inputs" (voltages)
      elevatorSim.setInputVoltage(motor.getAppliedVoltage().in(Volts));

      Time tick = SimulatedArena.getSimulationDt();
      motorSim.update(tick);
      elevatorSim.update(tick.in(Seconds));

      Logger.recordOutput("ElevatorIOSim/Setpoint", positionPid.getGoal().position);
      Logger.recordOutput("ElevatorIOSim/PIDOutput", pidOutput);
      Logger.recordOutput("ElevatorIOSim/FFOutput", feedforwardOutput);
      Logger.recordOutput(
          "ElevatorIOSim/Position",
          elevatorSim.getPositionMeters() / ElevatorConstants.kElevatorDrumRadius);
      Logger.recordOutput("ElevatorIOSim/SubTickNum", subTickNum);
      Logger.recordOutput(
          "ElevatorIOSim/Velocity",
          elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.kElevatorDrumRadius);
      Logger.recordOutput("ElevatorIOSim/Voltage", motor.getAppliedVoltage().in(Volts));
    }
  }
}
