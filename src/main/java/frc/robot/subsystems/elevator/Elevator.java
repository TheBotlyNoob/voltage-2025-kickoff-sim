package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {

  protected final ElevatorIO elevatorIo;
  protected final ElevatorIOInputsAutoLogged elevatorIoInputs = new ElevatorIOInputsAutoLogged();

  // Create a Mechanism2d visualization of the elevator
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(20, 50);
  private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
  private final LoggedMechanismLigament2d elevatorMech2d =
      mech2dRoot.append(
          new LoggedMechanismLigament2d("Elevator", elevatorIoInputs.positionRad, 90));

  public Elevator(ElevatorIO elevatorIo) {
    this.elevatorIo = elevatorIo;
  }

  public void setLevel(double level) {}

  @Override
  public void periodic() {
    elevatorIo.updateInputs(elevatorIoInputs);
    Logger.processInputs("Elevator", elevatorIoInputs);

    elevatorMech2d.setLength(elevatorIoInputs.positionRad);
    Logger.recordOutput("Elevator/Mechanism", mech2d);
  }
}
