package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.Swerve; // Ensure this matches your YAGSL swerve subsystem

//useless do not use this just test code 

public class AprilTagAligner
        extends SubsystemBase {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.kDefaultField);
    private static final double TARGET_DISTANCE_METERS = 0.91; // 3 feet in meters
    private final PhotonCamera camera;
    private final Swerve swerve;


    private final PIDController xController = new PIDController(0.8, 0, 0);
    private final PIDController yController = new PIDController(0.8, 0, 0);
    private final PIDController rotationController = new PIDController(0.05, 0, 0);
    public double xSpeed;
    public double ySpeed;
    public double rotationSpeed;

    public AprilTagAligner(String cameraName, Swerve swerve) {
        this.camera = new PhotonCamera(cameraName);
        this.swerve = swerve;

        // Set PID tolerances
        xController.setTolerance(0.05); // Forward/backward tolerance
        yController.setTolerance(0.02); // Left/right tolerance
        rotationController.setTolerance(2.0); // Rotation tolerance in degrees
    }

    public void alignToAprilTag() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
            return;
        }

        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d tagPose = target.getBestCameraToTarget();

        double xOffset = tagPose.getX(); // Distance error
        double yOffset = tagPose.getY(); // Sideways error
        Pose2d rotationError = (fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d());
        Rotation2d rotationErrorfix = rotationError.getRotation();
        Double rotationeror = rotationErrorfix.getRadians();
        double swervedriverotation = swerve.getPose().getRotation().getRadians() - rotationeror;
        xSpeed = xController.calculate(xOffset, 0); // Forward/back
        ySpeed = yController.calculate(yOffset, 0); // Strafe
        rotationSpeed = rotationController.calculate(swervedriverotation, 0); // Rotate
        System.out.println(xSpeed);
        System.out.println(ySpeed);
        System.out.println(rotationSpeed);
        //swerve.driveFieldOriented(new ChassisSpeeds(xSpeed, -ySpeed, rotationSpeed));
    }

    public double getXSpeed() {
        return xSpeed;
    }

    public double getYSpeed() {
        return ySpeed;
    }

    public double getrotationSpeed() {
        return rotationSpeed;
    }

    public void stop() {
        swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}