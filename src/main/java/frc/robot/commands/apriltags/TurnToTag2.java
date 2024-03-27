package frc.robot.commands.apriltags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
        double angleSpeed = 0.0;
        double forwardSpeed = 0.0;
        if (result.hasTargets()) {

            PhotonTrackedTarget mainTarget;
            List<PhotonTrackedTarget>targets = result.getTargets();
            for (PhotonTrackedTarget target : targets) 
            {
                int targetId = target.getFiducialId();
                if (targetId == 4 || targetId == 7)
                {
                    mainTarget = target;
                    angleSpeed = angle.calculate(mainTarget.getYaw(), 0);
                    double range = mainTarget.getBestCameraToTarget().getX();
                    forwardSpeed = forward.calculate(range, Constants.Shooter.GOAL_RANGE_METERS);
                    // System.out.println("Forward speed: " + forwardSpeed);
                    // System.out.println("Forward distance: " + range);
                    break;
                }
            }
        }
        swerveSubsystem.driveWithVision(angleSpeed, forwardSpeed);
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