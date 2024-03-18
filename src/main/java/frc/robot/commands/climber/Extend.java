package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PinSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class Extend extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final PinSubsystem pinSubsystem;

  public Extend(ClimberSubsystem climberSubsystem, PinSubsystem pinSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.pinSubsystem = pinSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem, pinSubsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Starting arm extension");
    }

    @Override
    public void execute() {
      climberSubsystem.climbArmUp();
      pinSubsystem.enablePin();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      System.out.println("Stopping arm extension");
      climberSubsystem.stopClimb();
      pinSubsystem.disablePin();
    }

  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
};