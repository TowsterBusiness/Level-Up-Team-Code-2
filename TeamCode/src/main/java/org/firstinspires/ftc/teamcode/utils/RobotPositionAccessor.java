package org.firstinspires.ftc.teamcode.utils;

public class RobotPositionAccessor {
  public static double[] getPosition(RobotPositions position) {
    switch (position) {
      case PICKUP:
        return new double[] { 1194, 829, 0.422 };
      case TRUSS:
        return new double[] { 2262, 627, 0.66 };
      case PLACE:
        return new double[] { 1795, 99, 0 };
      case HANG_START:
        return new double[] { 1280, 333 };
      case HANG_END:
        return new double[] { 529, 130 };
      default:
        return new double[] { 0, 0 };
    }
  }
}