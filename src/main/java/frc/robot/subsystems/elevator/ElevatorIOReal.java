package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SimConstants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

  private final SparkFlex leader;
  private final RelativeEncoder leaderEncoder;
  private final SparkFlex follower;

  private final ElevatorFeedforward feedforward;
  private final SparkClosedLoopController leaderController;

  private final DigitalInput topLimit = new DigitalInput(ElevatorConstants.kBottomLimitSwitch);
  private final DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.kTopLimitSwitch);

  private final Debouncer faultDebounce = new Debouncer(0.5);

  public ElevatorIOReal() {
    leader = new SparkFlex(ElevatorConstants.kElevatorLeaderCanId, MotorType.kBrushless);
    leaderEncoder = leader.getEncoder();
    leaderController = leader.getClosedLoopController();

    follower = new SparkFlex(ElevatorConstants.kElevatorFollowerCanId, MotorType.kBrushless);

    SparkBaseConfig leaderConf = new SparkFlexConfig();

    leaderConf
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .voltageCompensation(12.0);

    // leaderConf.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(ElevatorConstants.PID.kP, ElevatorConstants.PID.kI, ElevatorConstants.PID.kD, 0.0)
    leaderConf
        .closedLoop
        .maxMotion
        .maxAcceleration(ElevatorConstants.kMaxAcceleration)
        .maxVelocity(ElevatorConstants.kMaxVelocity)
        .allowedClosedLoopError(0.001)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    tryUntilOk(
        5,
        () ->
            leader.configure(
                leaderConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    if (atBottom()) {
      setEncoderPosition(0.0);
    }

    SparkBaseConfig followerConf = new SparkFlexConfig();
    followerConf
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .follow(leader, false);
    follower.configure(
        followerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.FF.kS,
            ElevatorConstants.FF.kG,
            ElevatorConstants.FF.kV,
            ElevatorConstants.FF.kA);

    new Trigger(topLimit::get).or(bottomLimit::get).onTrue(Commands.runOnce(() -> setVoltage(0.0)));
  }

  private boolean atBottom() {
    return !bottomLimit.get();
  }

  private boolean atTop() {
    return topLimit.get();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sparkStickyFault = false;
    RelativeEncoder leadEncoder = leader.getEncoder();
    ifOk(
        leader,
        leadEncoder::getPosition,
        (rot) -> inputs.positionRad = Units.rotationsToRadians(rot));
    ifOk(
        leader,
        leadEncoder::getVelocity,
        (vel) -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(vel));
    ifOk(leader, leader::getAppliedOutput, (volt) -> inputs.leadAppliedVolts = volt);
    ifOk(leader, leader::getOutputCurrent, (curr) -> inputs.leadCurrentAmps = curr);
    ifOk(follower, follower::getAppliedOutput, (volt) -> inputs.followAppliedVolts = volt);
    ifOk(follower, follower::getOutputCurrent, (curr) -> inputs.followCurrentAmps = curr);

    inputs.motorConnected = faultDebounce.calculate(!sparkStickyFault);

    inputs.atBottom = atBottom();
    inputs.atTop = atTop();
  }

  @Override
  public void setTargetPosition(double rads) {
    leaderController.setReference(
        rads,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(leaderEncoder.getVelocity()),
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setEncoderPosition(double rads) {
    tryUntilOk(5, () -> leaderEncoder.setPosition(Units.radiansToRotations(rads)));
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }
}
