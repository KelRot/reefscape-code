package frc.robot.utils;


import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Wrist.WristConstants;

public class RobotMath
{

  public static class ArmMath
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertArmAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) * ArmConstants.gearRatio);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToArmAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) / ArmConstants.gearRatio);

    }
  }

  public static class WristMath
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertWristAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) * WristConstants.gearRatio);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToWristAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) / WristConstants.gearRatio);

    }
  }
}