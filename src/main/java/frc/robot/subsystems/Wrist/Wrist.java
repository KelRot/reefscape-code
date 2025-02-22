package frc.robot.subsystems.Wrist;

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

public class Wrist extends SubsystemBase {
  private SparkMax m_motor;
  private WPI_VictorSPX m_wheelMotor;
  private double speed;
  private SparkClosedLoopController closedLoopController;
  private SparkMaxConfig motorConfig;
  private RelativeEncoder encoder;
  private double currentAngleSetpoint;

  public Wrist() {
    m_motor = new SparkMax(WristConstants.SparkID, MotorType.kBrushless);
    m_wheelMotor = new WPI_VictorSPX(WristConstants.RedlineID);
    encoder = m_motor.getEncoder();
    closedLoopController = m_motor.getClosedLoopController();
    motorConfig = new SparkMaxConfig();
    m_motor.configure(configCreator(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(WristConstants.defaultAngle);
    speed = 0.5;
  }

  public void setAngle(double angle) {
    if (isAngleInRange(angle)) {
      currentAngleSetpoint = angle;
      closedLoopController.setReference(angle * WristConstants.gearRatio, 
                                       ControlType.kMAXMotionPositionControl, 
                                       ClosedLoopSlot.kSlot0);
    } else {
      setDefault();
    }
  }

  public void setAngleTest() {
    double angle = SmartDashboard.getNumber("testAngleAnkle", 0);
    if (isAngleInRange(angle)) {
      closedLoopController.setReference(angle, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    } else {
      setDefault();
    }
  }

  public void setDefault() {
    closedLoopController.setReference(WristConstants.defaultAngle, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
    currentAngleSetpoint = 0;
    if (encoder.getPosition() == WristConstants.defaultAngle) {
      stopAngleMotor();
    }
  }

  public boolean isAngleInRange(double angle) {
    return angle <= WristConstants.maxAngle && angle >= WristConstants.minAngle;
  }


  public void stopAngleMotor() {
    m_motor.set(0);
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

  private SparkMaxConfig configCreator(SparkMaxConfig motorConfig) {
    String prefix = "ankle";
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
    motorConfig.smartCurrentLimit(40);

    return motorConfig;
  }

}
