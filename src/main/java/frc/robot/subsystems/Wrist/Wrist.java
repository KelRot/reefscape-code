package frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.utils.RobotMath.ArmMath;
import frc.robot.utils.RobotMath.WristMath;

public class Wrist extends SubsystemBase {
  private SparkMax masterMotor;
  private WPI_VictorSPX m_wheelMotor;
  private double speed;
  private SparkClosedLoopController closedLoopController;
  private SparkMaxConfig masterMotorConfig;
  private RelativeEncoder encoder;
  private double currentAngleSetpoint;
  private final MutAngle m_angle;
  private final MutAngularVelocity m_velocity;;

  public Wrist() {
    masterMotor = new SparkMax(ArmConstants.masterNeoID, MotorType.kBrushless);
    masterMotorConfig = new SparkMaxConfig();
    masterMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).voltageCompensation(12).closedLoop
        .pid(WristConstants.sparkKP, WristConstants.sparkKI, WristConstants.sparkKD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(-0.9, 0.9);
    masterMotorConfig.closedLoop.maxMotion.maxVelocity(WristConstants.maxRPM)
        .maxAcceleration(WristConstants.maxAccelaration)
        .allowedClosedLoopError(WristConstants.allowedClosedLoopError.in(Rotations));
    masterMotor.configure(masterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_angle = Rotations.mutable(0);
    m_velocity = RPM.mutable(0);
  }

  public void reachSetPoint(double angle) {
    double goalPosition = WristMath.convertWristAngleToSensorUnits(Degrees.of(angle)).in(Rotations);
    closedLoopController.setReference(goalPosition, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
  }

  public void setDefault() {
    double goalPosition = ArmMath.convertArmAngleToSensorUnits(Degrees.of(0)).in(Rotations);
    closedLoopController.setReference(goalPosition, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
  }

  public void stopAngleMotor() {
    masterMotor.set(0);
  }

  /*
   * private void setWheelMotor(double velocity) {
   * m_wheelMotor.set(velocity);
   * }
   */

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Angle (Ankle)", currentAngleSetpoint);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
