package frc.robot.subsystems.Arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import java.io.ObjectOutputStream.PutField;
import java.rmi.MarshalException;

import org.opencv.core.Mat;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;;

public class Arm extends SubsystemBase {

    private final SparkMax masterMotor, followerMotor;
    private final SparkMaxConfig followerMotorConfig, masterMotorConfig;
    private final boolean isRioPIDController;
    private PIDController pidController;
    //private final SparkClosedLoopController closedLoopController;
    private final Encoder quadEncoder;
    private final RelativeEncoder masterEncoder;
    private double lastKP, lastKI, lastKD, lastOutput = 0;

    public Arm() {
        masterMotor = new SparkMax(ArmConstants.masterNeoID, MotorType.kBrushless);
        followerMotor = new SparkMax(ArmConstants.followerNeoID, MotorType.kBrushless);
        masterMotorConfig = new SparkMaxConfig();
        followerMotorConfig = new SparkMaxConfig();
        masterEncoder = masterMotor.getEncoder();
        masterMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).voltageCompensation(12);
        followerMotorConfig.smartCurrentLimit(40).follow(masterMotor).voltageCompensation(12).idleMode(IdleMode.kBrake);
        followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        masterMotor.configure(masterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        isRioPIDController = SmartDashboard.getBoolean("isRioPIDController", true);
        pidController = new PIDController(ArmConstants.rioKP, ArmConstants.rioKI, ArmConstants.rioKD);
        pidController.setTolerance(0.01);
        //closedLoopController = masterMotor.getClosedLoopController();
        quadEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        quadEncoder.reset();
        quadEncoder.setDistancePerPulse(0.6);
        SmartDashboard.putNumber("Arm/SetPoint", getAngle());
    }

    public void reachSetPoint(double angle) { 
        if (isRioPIDController)
        {
          double pidOutput     = pidController.calculate(getAngle(), angle);
          double direction = pidOutput > 0 ? 1 : -1;
          pidOutput = direction * Math.min(1.37, Math.abs(pidOutput));
          lastOutput = pidOutput;
          masterMotor.setVoltage(pidOutput + getFeedForward(getAngle()));
          System.out.println(angle);
          SmartDashboard.putNumber("pidoutput", pidOutput);
          SmartDashboard.putNumber("voltage angle", getAngle());
        } else
        {
       //   closedLoopController.setReference(angle,
       //                             ControlType.kPosition, ClosedLoopSlot.kSlot0, getFeedForward(getAngle()));
        }
    }
    public double getFeedForward(double angleInDegrees) { // Calculates The Feed Forward Value
        double direction = angleInDegrees > 0 ? -1 : 1;
        return direction * (0.85 * Math.abs(Math.sin(Math.toRadians(Math.abs(angleInDegrees)))));
    }
    public void setSetPoint(double Angle) {
        SmartDashboard.putNumber("Arm/SetPoint", Angle);
    }

    public double getAngle() { // Get Angle Value From Encoders
        return (quadEncoder.getDistance() - ArmConstants.defaultAngle);
    }

    public void setVoltage() {
        double num = SmartDashboard.getNumber("Arm/Debug Voltage", 0);
        masterMotor.setVoltage(num);
    }
    public PIDController refreshPidController() {
        if (SmartDashboard.getNumber("Arm/RioKP", ArmConstants.rioKP) != lastKP
                || SmartDashboard.getNumber("Arm/RioKI", ArmConstants.rioKI) != lastKI
                || SmartDashboard.getNumber("Arm/RioKD", ArmConstants.rioKD) != lastKD) {
                    lastKP = SmartDashboard.getNumber("Arm/RioKP", ArmConstants.rioKP);
                    lastKI = SmartDashboard.getNumber("Arm/RioKI", ArmConstants.rioKP);
                    lastKD = SmartDashboard.getNumber("Arm/RioKD", ArmConstants.rioKP);
            return new PIDController(SmartDashboard.getNumber("Arm/RioKP", ArmConstants.rioKP),
                    SmartDashboard.getNumber("Arm/RioKI", ArmConstants.rioKI),
                    SmartDashboard.getNumber("Arm/RioKD", ArmConstants.rioKD));
        } else {
            return new PIDController(lastKP, lastKI, lastKD);
        }
    }


    @Override
    public void periodic() {
        masterEncoder.setPosition(quadEncoder.getDistance());
        SmartDashboard.putNumber("Arm/Get Angle", getAngle());
        SmartDashboard.putNumber("Arm/Get", quadEncoder.getDistance());
        SmartDashboard.putNumber("Arm/Neo Encoder", masterEncoder.getPosition());
        SmartDashboard.putNumber("voltage", lastOutput + getFeedForward(getAngle()));
        SmartDashboard.putNumber("Applied Voltage", masterMotor.get());
        if (SmartDashboard.getNumber("Arm/Debug Voltage", 0) != 0) {
         setVoltage();
        } else {
            double angle = SmartDashboard.getNumber("Arm/SetPoint", 0);
            reachSetPoint(angle);
        }
        pidController = refreshPidController();
    }

    @Override
    public void simulationPeriodic() {
    }
}