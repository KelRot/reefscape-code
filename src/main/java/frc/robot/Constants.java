package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class GeneralConstants {
    public static final int LED_LENGTH = 73;
    public static final int LED_PWM_PORT = 8;
  }
  public static class LevelAngles {
    public static final int Level1 = -75;
    public static final int Level2 = -68;
    public static final int FrontLevel3 = -20; 
    public static final int BackLevel3 = -3;
    public static final double DefaultAngle = -82.65;
    public static final double DefaultAngleWrist = -12;
    public static final double BackLevel3Wrist = -12;
    public static final double BackAlgaeRemover = 0;
    public static final double BackAlgaeRemoverWrist = 0;
  }
}
