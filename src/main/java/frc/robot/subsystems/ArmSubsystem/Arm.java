package frc.robot.subsystems.ArmSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final SparkMax m_motor;
    private final SparkMax m_followerMotor;
    private final SparkMaxConfig followerMotorConfig, m_motorConfig;
    // Ayarlanan açı
    private double currentAngle;

    public Arm() {
        m_motor = new SparkMax(ArmConstants.masterNeoID, MotorType.kBrushless);
        m_followerMotor = new SparkMax(ArmConstants.followerNeoID, MotorType.kBrushless);
        followerMotorConfig = new SparkMaxConfig();
        m_motorConfig = new SparkMaxConfig();
        followerMotorConfig.follow(m_motor);
        m_motor.configure(configCreator(m_motorConfig), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        currentAngle = 0;
    }

    // Arm'ı belirli bir açıya ayarlama fonksiyonu
    private void setAngle(double angle) {
        if (isAngleInRange(angle)) {
            currentAngle = angle;

        } else {
            stopMotor();
        }
    }

    // Test için arm'ı sabit bir açıya ayarlama
    private void setAngleTest() {
        double testAngle = SmartDashboard.getNumber("testAngle", 0);
        setAngle(testAngle);
    }

    // Varsayılan pozisyona geri dönme (0 derece)
    private void setDefault() {
        setAngle(ArmConstants.defaultAngle);
    }

    private boolean isAngleInRange(double angle) {
        return angle >= ArmConstants.defaultAngle && angle <= ArmConstants.maxAngle;
    }

    // Motoru durdurma
    private void stopMotor() {
        m_motor.setVoltage(0);
        m_followerMotor.setVoltage(0);
    }

    // Robotun her döngüsünde çalışacak fonksiyon
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Angle (Arm)", currentAngle);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private SparkMaxConfig configCreator(SparkMaxConfig motorConfig) {
    String prefix = "arm";
    double kP = SmartDashboard.getNumber(prefix + "P", ArmConstants.kP);
    double kI = SmartDashboard.getNumber(prefix + "I", ArmConstants.kI);
    double kD = SmartDashboard.getNumber(prefix + "D", ArmConstants.kD);
    double kMinOutput = SmartDashboard.getNumber(prefix + "MinOutput", ArmConstants.MinOutput);
    double kMaxOutput = SmartDashboard.getNumber(prefix + "MaxOutput", ArmConstants.MaxOutput);
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
        return motorConfig;
  }

}