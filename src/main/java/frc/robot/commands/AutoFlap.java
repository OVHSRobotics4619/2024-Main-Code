package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlapSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class AutoFlap extends Command {

    private final FlapSubsystem flapSubsystem;
    private Timer driveTime = new Timer();
    private double Time;

  public AutoFlap(FlapSubsystem flapSubsystem) {
    this.flapSubsystem = flapSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.flapSubsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Flapping!");
      
      driveTime.reset();
      driveTime.start();
    }

    @Override
    public void execute() {
      Time = driveTime.get();
      flapSubsystem.enableFlap();
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
      flapSubsystem.disableFlap();
    }

  
    // Returns true when the command should end
    @Override
    public boolean isFinished() {
      if (Time > 0.5) {
        return true;
      }
      return false;
    }
};