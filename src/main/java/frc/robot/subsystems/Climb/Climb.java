package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climb extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax opener_motor, closer_motor; 
  private final SparkClosedLoopController openerPID, closerPID;
  private final SparkMaxConfig motorConfig;
  // I use here PID cuz when setup this type of closedloopcontroller increases motor torque.
  public Climb() {
    opener_motor = new SparkMax(ClimbConstants.openerNeoID, MotorType.kBrushless);
    closer_motor = new SparkMax(ClimbConstants.closerNeoID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
    motorConfig.closedLoop.maxMotion.maxVelocity(25).maxAcceleration(5).allowedClosedLoopError(Degrees.of(0.01).in(Rotation));
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0, ClosedLoopSlot.kSlot1).outputRange(-1, 1);
    motorConfig.closedLoop.maxMotion.maxVelocity(50).maxAcceleration(5).allowedClosedLoopError(Degrees.of(0.01).in(Rotation));
    motorConfig.smartCurrentLimit(40);
    openerPID = opener_motor.getClosedLoopController();
    closerPID = closer_motor.getClosedLoopController();
    opener_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    closer_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }
  private void openClimb() {
    openerPID.setReference(Degrees.of(120).in(Rotation) * ClimbConstants.openerGearRatio, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }
  private void closeClimb() {
    closerPID.setReference(0 * ClimbConstants.closerGearRatio, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
