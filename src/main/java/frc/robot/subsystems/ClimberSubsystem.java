package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class ClimberSubsystem extends SubsystemBase {
  private TalonSRX climbMotor = new TalonSRX(16);

  public ClimberSubsystem() {
    this.climbMotor.setInverted(true);
  }

  public void climb(double speed) {

    // Positive lifts robot up, negative puts robot down
    climbMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void climbArmUp() {
    this.climb(Constants.Climber.CLIMB_SPEED);
  }

  public void climbArmDown() {
    this.climb(-Constants.Climber.CLIMB_SPEED);
  }

  public void stopClimb() {
    climbMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }
}