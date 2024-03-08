package frc.robot.commands.apriltags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class TurnToTag2 extends Command {

    private final PhotonCamera camera;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController angle;
    private final PIDController forward;

    public TurnToTag2(PhotonCamera camera, SwerveSubsystem swerveSubsystem, PIDController angle, PIDController forward) {
        this.camera = camera;
        this.angle = angle;
        this.forward = forward;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization logic (if needed)
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            double angleSpeed = angle.calculate(result.getBestTarget().getYaw(), 0);
            double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.AprilTags.CameraConstants.CAMERA_HEIGHT_METERS, Constants.AprilTags.CameraConstants.TARGET_HEIGHT_METERS, Constants.AprilTags.CameraConstants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            double forwardSpeed = -forward.calculate(range, Constants.AprilTags.CameraConstants.GOAL_RANGE_METERS);
            swerveSubsystem.driveWithVision(angleSpeed);
        } else {
            double angleSpeed = 0.0;
            double forwardSpeed = 0.0;
            swerveSubsystem.driveWithVision(angleSpeed);
        }

    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup logic (if needed)
    }

    @Override
    public boolean isFinished() {
        // Determine when the command should end (e.g., after a certain duration)
        return false;
    }
}