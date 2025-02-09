package frc.robot.subsystems.AnkleSubsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AnkleSubsystem extends SubsystemBase {
  private SparkMax m_motor;
  private WPI_VictorSPX m_wheelMotor;
  private SparkClosedLoopController closedLoopController;
  private SparkMaxConfig motorConfig;
  private RelativeEncoder encoder;
  public double kP, kI, kD, kIz, kMaxOutput, kMinOutput;

  public AnkleSubsystem() {
    m_motor = new SparkMax(AnkleConstants.SparkID, MotorType.kBrushless);
    m_wheelMotor = new WPI_VictorSPX(AnkleConstants.RedlineID);
    encoder = m_motor.getEncoder();
    closedLoopController = m_motor.getClosedLoopController();
    motorConfig = new SparkMaxConfig();
    configureMotorConfig(motorConfig);
    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void setAngle(double angle) {
    if (isAngleInRange(angle)) {
      closedLoopController.setReference(angle * AnkleConstants.gearRatio, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    } else {
      setDefault();
    }
  }

  public void setAngleTest() {
    double angle = SmartDashboard.getNumber("testAngleAnkle", 0) * AnkleConstants.gearRatio;
    if (isAngleInRange(angle)) {
      closedLoopController.setReference(angle, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    } else {
      setDefault();
    }
  }

  public void setDefault() {
    closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
    if (encoder.getPosition() == 0) {
      stopAngleMotor();
    }
  }

  public boolean isAngleInRange(double angle) {
    return angle <= AnkleConstants.maxAngle && angle >= AnkleConstants.minAngle;
  }

  public void stopAngleMotor() {
    m_motor.set(0);
  }

  public void setWheelMotor(double velocity) {
    m_wheelMotor.set(velocity);
  }

  @Override
  public void periodic() {
    encoder.setPosition(0);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void configureMotorConfig(SparkMaxConfig motorConfig) {
    String prefix = "ankle";
    double kP = SmartDashboard.getNumber(prefix + "P", AnkleConstants.kP);
    double kI = SmartDashboard.getNumber(prefix + "I", AnkleConstants.kI);
    double kD = SmartDashboard.getNumber(prefix + "D", AnkleConstants.kD);
    double kMinOutput = SmartDashboard.getNumber(prefix + "MinOutput", AnkleConstants.MinOutput);
    double kMaxOutput = SmartDashboard.getNumber(prefix + "MaxOutput", AnkleConstants.MaxOutput);
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
  }

}
