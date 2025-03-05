package frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private TalonFX masterMotor;
  private WPI_VictorSPX m_wheelMotor;
  private double currentAngleSetpoint;
  private TalonFXConfiguration masterConfig;
  public Wrist() {
    masterMotor = new TalonFX(WristConstants.FalconID, "rio");
    masterConfig = new TalonFXConfiguration();
    masterConfig.Voltage.withPeakForwardVoltage(12).withPeakReverseVoltage(-12);
    m_wheelMotor = new WPI_VictorSPX(5);
  }

  public void setVoltage() {
    double num = SmartDashboard.getNumber("Wrist/Debug Voltage", 0);
    masterMotor.setVoltage(num);
}
  public void setWheelMotor(double volts) {
    m_wheelMotor.setVoltage(volts);
   }

  public void stopAngleMotor() {
    masterMotor.set(0);
  }
  
   
   

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Angle (Ankle)", currentAngleSetpoint);
    setVoltage();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
