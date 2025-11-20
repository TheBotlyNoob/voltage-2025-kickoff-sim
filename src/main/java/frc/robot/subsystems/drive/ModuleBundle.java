package frc.robot.subsystems.drive;

public class ModuleBundle<T> {

  public final T frontLeft;
  public final T frontRight;
  public final T backLeft;
  public final T backRight;

  public ModuleBundle(T frontLeft, T frontRight, T backLeft, T backRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
  }

  public T[] toArray() {
    return (T[]) new Object[] {frontLeft, frontRight, backLeft, backRight};
  }
}
