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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.RobotStatusManager;
import frc.robot.subsystems.RobotStatusManager.RobotStatus;



public class Climb extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax opener_motor, closer_motor; 
  private final SparkClosedLoopController openerPID, closerPID;
  private final SparkMaxConfig motorConfig;
  private final RobotStatusManager robotStatusManager;
  // I use here PID cuz when setup this type of closedloopcontroller increases motor torque.
  public Climb() {
    opener_motor = new SparkMax(ClimbConstants.openerNeoID, MotorType.kBrushless);
    closer_motor = new SparkMax(ClimbConstants.closerNeoID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(40);
    openerPID = opener_motor.getClosedLoopController();
    closerPID = closer_motor.getClosedLoopController();
    robotStatusManager = new RobotStatusManager();
    opener_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    closer_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void openClimb() {
    robotStatusManager.setStatus(RobotStatus.Climbing);
    opener_motor.set(0.6);
  }
  public void closeClimb() {
    closer_motor.set(1);
  }

  @Override
  public void periodic() { 
    /*if(SmartDashboard.getNumber("Climb/Opener Set", 0) != 0 || SmartDashboard.getNumber("Climb/Closer Set", 0) != 0) {
      setOpener();
      setCloser();
    }*/
  }

  @Override
  public void simulationPeriodic() {
  }

public void stopOpener() {
   opener_motor.set(0);
}
public void stopCloser() {
  closer_motor.set(0);
}
public void setOpener() {
  double num = SmartDashboard.getNumber("Climb/Opener Set", 0);
  opener_motor.set(num);
}
public void setCloser(double num) {
  closer_motor.set(num);
}
}
