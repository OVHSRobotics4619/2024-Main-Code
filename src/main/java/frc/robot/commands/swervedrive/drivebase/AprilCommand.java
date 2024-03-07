package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AprilCommand extends Command {

    private final PhotonCamera camera;
    private final SwerveSubsystem swerve;

    private double cameraYaw = 10; // default value for the yaw; arbitrary

    public AprilCommand(PhotonCamera camera, SwerveSubsystem swerve) {
        this.camera = camera;
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Initialization logic (if needed)
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            double yaw = result.getBestTarget().getYaw();

            this.cameraYaw = yaw;
            //System.out.println(yaw);

            double rotationSpeed = -yaw * (Math.PI / 180) * Constants.AprilTags.TURN_SPEED_MULTIPLIER;
            
            swerve.driveWithVision(rotationSpeed);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Cleanup logic (if needed)
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(this.cameraYaw) < Constants.AprilTags.TURN_STOP_LIMIT) {
            return true;
        } else {
            return false;
        }
    }
}