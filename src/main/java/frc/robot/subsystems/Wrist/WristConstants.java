package frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.RobotMath.WristMath;

public class WristConstants {
    public final static int RedlineID = 1;
    public final static int SparkID = 1;
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
    public static final Angle allowedClosedLoopError = WristMath.convertWristAngleToSensorUnits(Degrees.of(0.01));
}
