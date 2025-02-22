package frc.robot.subsystems.Climb;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Wrist.WristConstants;



public class Climb extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax opener_motor, closer_motor; 
  private final WPI_VictorSPX m_redline;
  private final SparkMaxConfig openerConfig, closerConfig;
  public Climb() {
    m_redline = new WPI_VictorSPX(ClimbConstants.RedlineID);
    opener_motor = new SparkMax(ClimbConstants.openerNeoID, MotorType.kBrushless);
    closer_motor = new SparkMax(ClimbConstants.closerNeoID, MotorType.kBrushless);
    openerConfig = new SparkMaxConfig();
    closerConfig = new SparkMaxConfig();
    opener_motor.configure(configCreator(openerConfig), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    closer_motor.configure(configCreator(closerConfig), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
    private SparkMaxConfig configCreator(SparkMaxConfig motorConfig) {
    String prefix = "climb";
    double kP = SmartDashboard.getNumber(prefix + "P", WristConstants.kP);
    double kI = SmartDashboard.getNumber(prefix + "I", WristConstants.kI);
    double kD = SmartDashboard.getNumber(prefix + "D", WristConstants.kD);
    double kMinOutput = SmartDashboard.getNumber(prefix + "MinOutput", WristConstants.MinOutput);
    double kMaxOutput = SmartDashboard.getNumber(prefix + "MaxOutput", WristConstants.MaxOutput);
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(kMinOutput, kMaxOutput);

    motorConfig.closedLoop.maxMotion
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1);
        motorConfig.smartCurrentLimit(50);

        return motorConfig; // todo opener pid closer just voltage b-o-t just add current limit 
  }
}
