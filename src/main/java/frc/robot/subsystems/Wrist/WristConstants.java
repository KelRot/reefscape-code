package frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class WristConstants {
    public final static int RedlineID = 5;
    public final static int FalconID = 0;
    public final static double maxAngle = 1;
    public final static double minAngle = 100;
    public static final double MaxOutput = 0;
    public static final double MinOutput = 0;    
    public static double maxRPM = 20;
    public static double maxAccelaration = 0.7;
    public static double sparkKD;
    public static double sparkKI;
    public static double sparkKP;
    public static double gearRatio = 4;
    public static double defaultAngle = 20;
    public static final Angle allowedClosedLoopError = Degrees.of(1);
}
