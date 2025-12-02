package frc.robot.subsystems.drive;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.PhoenixUtil;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Max drive motor controller, Spark Max turn motor controller,
 * and CANcoder absolute encoder.
 * <p>
 * Does not support high-frequency encoding
 */
public class ModuleIOSparkMaxCancoder implements ModuleIO {

  public interface Constants {

    Rotation2d[] zeroRotations();

    int[] driveCanIds();

    int[] turnCanIds();

    int driveCurrentLimit();

    int turnCurrentLimit();

    double driveKp();

    double driveKd();

    double driveKs();

    double driveKv();

    double driveEncoderPositionFactor();

    double driveEncoderVelocityFactor();

    double turnKp();

    double turnKd();

    double turnPIDMinInput();

    double turnPIDMaxInput();

    double turnEncoderPositionFactor();

    double turnEncoderVelocityFactor();

    boolean turnInverted();

    boolean turnEncoderInverted();

    double odometryFrequency();
  }

  private final Constants consts;

  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final CANcoder turnEncoder;

  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final PIDController turnController;

  private boolean turnClosedLoop = true;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSparkMaxCancoder(Constants consts, int module) {
    this.consts = consts;

    zeroRotation = consts.zeroRotations()[module];
    driveSpark = new SparkFlex(consts.driveCanIds()[module], MotorType.kBrushless);
    turnSpark = new SparkMax(consts.turnCanIds()[module], MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(consts.driveCurrentLimit())
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(consts.driveEncoderPositionFactor())
        .velocityConversionFactor(consts.driveEncoderVelocityFactor())
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            consts.driveKp(), 0.0,
            consts.driveKd(), 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / consts.odometryFrequency()))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(consts.turnInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(consts.turnCurrentLimit())
        .voltageCompensation(12.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / consts.odometryFrequency()))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // configure turn CANcoder and PID controller
    turnController = new PIDController(consts.turnKp(), 0, consts.turnKd());
    turnEncoder = new CANcoder(-1);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    // TODO: ensure this is right.
    turnEncoderConfig.MagnetSensor.SensorDirection =
        consts.turnEncoderInverted()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> turnEncoder.getConfigurator().apply(turnEncoderConfig, 0.25));

    turnPosition = turnEncoder.getPosition();
    turnVelocity = turnEncoder.getVelocity();

    turnPosition.setUpdateFrequency(50.0);
    turnVelocity.setUpdateFrequency(50.0);

    turnEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnPosition, turnVelocity);

    if (turnClosedLoop) {
      turnSpark.setVoltage(
          turnController.calculate(
              turnPosition.getValueAsDouble() * consts.turnEncoderPositionFactor()));
    }

    // Update turn inputs
    sparkStickyFault = false;
    inputs.turnPosition =
        new Rotation2d(turnPosition.getValueAsDouble() * consts.turnEncoderPositionFactor())
            .minus(zeroRotation);
    inputs.turnVelocityRadPerSec =
        turnVelocity.getValueAsDouble() * consts.turnEncoderVelocityFactor();
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        consts.driveKs() * Math.signum(velocityRadPerSec) + consts.driveKv() * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(),
            consts.turnPIDMinInput(),
            consts.turnPIDMaxInput());
    turnController.setSetpoint(setpoint);
  }
}
