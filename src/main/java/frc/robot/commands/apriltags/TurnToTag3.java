package frc.robot.commands.apriltags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class TurnToTag3 extends Command {

    private final PhotonCamera camera;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController angle;
    private final PIDController forward;

    private double previousForwardSpeed;

    private boolean endCommand;

    private Timer driveTime = new Timer();
    private double Time;

    public TurnToTag3(PhotonCamera camera, SwerveSubsystem swerveSubsystem, PIDController angle, PIDController forward) {
        this.camera = camera;
        this.angle = angle;
        this.forward = forward;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        driveTime.reset();
        driveTime.start();

        previousForwardSpeed = 0;
        endCommand = false;
        // Initialization logic (if needed)
    }

    @Override
    public void execute() {
        Time = driveTime.get();

        PhotonPipelineResult result = camera.getLatestResult();
        double angleSpeed = 0.0;
        double forwardSpeed = previousForwardSpeed / 2;
        if (result.hasTargets()) {

            PhotonTrackedTarget mainTarget;
            List<PhotonTrackedTarget>targets = result.getTargets();
            for (PhotonTrackedTarget target : targets) 
            {
                int targetId = target.getFiducialId();
                if (targetId == 4 || targetId == 7)
                {

                    mainTarget = target;

                    Transform3d bestTagPose = mainTarget.getBestCameraToTarget();

                    angleSpeed = angle.calculate(mainTarget.getYaw(), 0);
                    double range = Math.sqrt(Math.pow(bestTagPose.getX(), 2) + Math.pow(bestTagPose.getY(), 2));
                    forwardSpeed = forward.calculate(range, Constants.Shooter.GOAL_RANGE_METERS);
                    previousForwardSpeed = forwardSpeed;

                    if ((Math.abs(Constants.Shooter.GOAL_RANGE_METERS - range) < 0.1) && (Math.abs(forwardSpeed) < 0.1)) {
                        endCommand = true;
                    }

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
        if (endCommand || (Time > 5)) {
            System.out.println("Stopped auto shooter alignment");
            return true;
        }
        return false;
    }
}