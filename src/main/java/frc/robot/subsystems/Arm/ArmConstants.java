package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.RobotMath.ArmMath;

public class ArmConstants {
    public final static int masterNeoID = 0;
    public final static int followerNeoID = 0;
    public final static double defaultAngle = 0;
    public final static double maxAngle = Degrees.of(274.165 - 19.165).in(Rotation);
    public final static double minAngle = Degrees.of(0).in(Rotation);
    public final static int gearRatio = 24;
    public static final double MaxOutput = 0;
    public static final double MinOutput = 0;
    public static final Angle allowedClosedLoopError = ArmMath.convertArmAngleToSensorUnits(Degrees.of(0.01));
    public static double rioKP = 0;
    public static double rioKI = 0;
    public static double rioKD = 0;
    public static double kG;
    public static double kV;
    public static double kS;
    public static double maxRPM = 20;
    public static double maxAccelaration = 0.7;
    public static double sparkKD;
    public static double sparkKI;
    public static double sparkKP;
}

