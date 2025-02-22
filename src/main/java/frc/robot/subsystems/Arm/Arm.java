package frc.robot.subsystems.Arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMath.ArmMath;

public class Arm extends SubsystemBase {

    private final SparkMax masterMotor, followerMotor;
    private final SparkMaxConfig followerMotorConfig, masterMotorConfig;
    private final boolean isRioPIDController;
    private final ProfiledPIDController pidController;
    private final SparkClosedLoopController closedLoopController;
    private final Encoder quadEncoder;
    private final ArmFeedforward feedForward;
    private final MutAngle m_angle;
    private final MutAngularVelocity m_velocity;

    public Arm() {
        masterMotor = new SparkMax(ArmConstants.masterNeoID, MotorType.kBrushless);
        followerMotor = new SparkMax(ArmConstants.followerNeoID, MotorType.kBrushless);
        masterMotorConfig = new SparkMaxConfig();
        followerMotorConfig = new SparkMaxConfig();
        masterMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).voltageCompensation(12).closedLoop
                .pid(ArmConstants.sparkKP, ArmConstants.sparkKI, ArmConstants.sparkKD).feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder).outputRange(-0.9, 0.9);
        masterMotorConfig.closedLoop.maxMotion.maxVelocity(ArmConstants.maxRPM)
                .maxAcceleration(ArmConstants.maxAccelaration)
                .allowedClosedLoopError(ArmConstants.allowedClosedLoopError.in(Rotations));
        followerMotorConfig.smartCurrentLimit(40).follow(masterMotor).voltageCompensation(12).idleMode(IdleMode.kBrake);
        followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        masterMotor.configure(masterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        isRioPIDController = SmartDashboard.getBoolean("isRioPIDController", true);
        pidController = new ProfiledPIDController(ArmConstants.rioKP, ArmConstants.rioKI, ArmConstants.rioKD,
                new Constraints(ArmConstants.maxRPM, ArmConstants.maxAccelaration));
        pidController.setTolerance(0.01);
        feedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
        closedLoopController = masterMotor.getClosedLoopController();
        quadEncoder = new Encoder(1, 2);
        quadEncoder.reset();
        m_angle = Rotations.mutable(0);
        m_velocity = RPM.mutable(0);
    }

    public void reachSetPoint(double angle) {
        double goalPosition = ArmMath.convertArmAngleToSensorUnits(Degrees.of(angle)).in(Rotations);
        if (isRioPIDController) {
            double pidOutput = pidController.calculate(quadEncoder.get(), goalPosition);
            State setpointState = pidController.getSetpoint();
            masterMotor.setVoltage(pidOutput +
                    feedForward.calculate(setpointState.position,
                            setpointState.velocity));
        } else {
            closedLoopController.setReference(goalPosition, ControlType.kMAXMotionPositionControl,
                    ClosedLoopSlot.kSlot0);
        }
    }

    public void setDefault() {
        double goalPosition = ArmMath.convertArmAngleToSensorUnits(Degrees.of(0)).in(Rotations);
        if (isRioPIDController) {
            double pidOutput = pidController.calculate(quadEncoder.get(), goalPosition);
            State setpointState = pidController.getSetpoint();
            masterMotor.setVoltage(pidOutput +
                    feedForward.calculate(setpointState.position,
                            setpointState.velocity));
        } else {
            closedLoopController.setReference(goalPosition, ControlType.kMAXMotionPositionControl,
                    ClosedLoopSlot.kSlot0);
        }
    }

    public void setVoltage() {
        double num = SmartDashboard.getNumber("armDebugVoltage", 0);
        masterMotor.setVoltage(num);
    }

    public void stopMotors() {
        masterMotor.set(0);
    }

    public Angle getAngle() {
        m_angle.mut_replace(ArmMath.convertSensorUnitsToArmAngle(m_angle.mut_replace(quadEncoder.get(),
                Rotations)));
        return m_angle;
    }

    public AngularVelocity getVelocity() {
        m_velocity.mut_replace(ArmMath.convertArmAngleToSensorUnits(Rotations.of(quadEncoder.getRate()))
                .per(Minute));
        return m_velocity;
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {
    }
}