package frc.robot.subsystems.Wrist;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private TalonFX masterMotor;
  private WPI_VictorSPX m_wheelMotor;
  private double currentAngleSetpoint;
  private TalonFXConfiguration masterConfig;
  private final DigitalInput m_sensor;
  private double lastOutput, lastKP, lastKD, lastKI;
  private PIDController pidController;
  
    public Wrist() {
      masterMotor = new TalonFX(WristConstants.FalconID, "rio");
      m_wheelMotor = new WPI_VictorSPX(5);
      m_sensor = new DigitalInput(2);
      masterMotor.setPosition(0);
      pidController = new PIDController(WristConstants.rioKP, WristConstants.rioKI, WristConstants.rioKD);
      pidController.setTolerance(1);
      
    }
    public PIDController refreshPidController() {
      if (SmartDashboard.getNumber("Wrist/RioKP", WristConstants.rioKP) != lastKP
              || SmartDashboard.getNumber("Wrist/RioKI", WristConstants.rioKI) != lastKI
              || SmartDashboard.getNumber("Wrist/RioKD", WristConstants.rioKD) != lastKD) {
                  lastKP = SmartDashboard.getNumber("Wrist/RioKP", WristConstants.rioKP);
                  lastKI = SmartDashboard.getNumber("Wrist/RioKI", WristConstants.rioKP);
                  lastKD = SmartDashboard.getNumber("Wrist/RioKD", WristConstants.rioKP);
          return new PIDController(SmartDashboard.getNumber("Wrist/RioKP", WristConstants.rioKP),
                  SmartDashboard.getNumber("Wrist/RioKI", WristConstants.rioKI),
                  SmartDashboard.getNumber("Wrist/RioKD", WristConstants.rioKD));
      } else {
          return new PIDController(lastKP, lastKI, lastKD);
      }
  }
  
    public void setVoltage() {
      double num = SmartDashboard.getNumber("Wrist/Debug Voltage", 0);
      masterMotor.setVoltage(num);
  }
    public void setFeedForward(double num) {
     masterMotor.setVoltage(num);
  }
    public void setWheelMotor(double volts) {
      m_wheelMotor.setVoltage(volts);
     }
  
    public void stopAngleMotor() {
      masterMotor.set(0);
    }
    
    public boolean getSensor() { 
      return m_sensor.get();
    }
    public double getAngle() {
      return Rotations.of(masterMotor.getPosition().getValueAsDouble()).in(Degrees) / 9.62962962962963 + 15;
    }
    
    public double getRealAngle() { 
      return -(SmartDashboard.getNumber("Arm/Get Angle", Constants.LevelAngles.DefaultAngle) + -13) - getAngle() - 93;
    }
     
    public double getFeedForward(double angleInDegrees) { // Calculates The Feed Forward Value
      double direction = angleInDegrees < 0 ? -1 : 1;
      return direction * (0.93 * Math.abs(Math.sin(Math.toRadians(Math.abs(angleInDegrees)))));
  }
  
    public void reachSetPoint(double angle) {
      double pidOutput     = pidController.calculate(getAngle(), angle);
      double direction = pidOutput < 0 ? 1 : -1;
      pidOutput = direction * Math.min(1.37, Math.abs(pidOutput));
      lastOutput = pidOutput;
      masterMotor.setVoltage(-pidOutput + getFeedForward(getRealAngle()));
      System.out.println(angle);
      SmartDashboard.putNumber("Wrist/pidoutput", -pidOutput);
    }
    public void setSetPoint(double Angle) {
      SmartDashboard.putNumber("Wrist/SetPoint", Angle);
  }
  
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Current Angle (Ankle)",  currentAngleSetpoint);
      SmartDashboard.putBoolean("Wrist/Sensor", getSensor());
      SmartDashboard.putNumber("Wrist/Get Angle", getAngle());
      SmartDashboard.putNumber("Wrist/Real Angle", getRealAngle());
      SmartDashboard.putNumber("Wrist/Output", getFeedForward(getRealAngle()));
      if(SmartDashboard.getNumber("Wrist/Debug Voltage", 0) != 0) {
        setFeedForward(getRealAngle());
      } else {
        double num = SmartDashboard.getNumber("Wrist/SetPoint", -12);
        reachSetPoint(num);
      }
      pidController = refreshPidController();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
