package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.units.measure.Angle;

public class ArmConstants {
    public final static int masterNeoID = 5;
    public final static int followerNeoID = 4;
    public final static double defaultAngle = 82.65;
    public final static double maxAngle = Degrees.of(274.165 - 19.165).in(Rotation);
    public final static double minAngle = Degrees.of(0).in(Rotation);
    public final static double gearRatio = 35.45142857142857;
    public static final double MaxOutput = 1;
    public static final double MinOutput = -1;
    public static final double allowedClosedLoopError = 2;
    public static double rioKP = 0.0454;
    public static double rioKI = 0;
    public static double rioKD = 0.000001;
    public static double maxRPM = 1000000000;
    public static double maxAccelaration = 100000000;
    public static double sparkKD = 0;
    public static double sparkKI = 0;
    public static double sparkKP = 0;
}

// -68 L2 / 