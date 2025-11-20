package frc.robot.util;

@FunctionalInterface
public interface IOFactory<C, IO> {

  IO create(C constants);
}
