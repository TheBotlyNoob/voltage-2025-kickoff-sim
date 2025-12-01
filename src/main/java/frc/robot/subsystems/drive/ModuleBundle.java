package frc.robot.subsystems.drive;

public record ModuleBundle<T>(T frontLeft, T frontRight, T backLeft, T backRight) {

  public T[] toArray() {
    return (T[]) new Object[] {frontLeft, frontRight, backLeft, backRight};
  }
}
