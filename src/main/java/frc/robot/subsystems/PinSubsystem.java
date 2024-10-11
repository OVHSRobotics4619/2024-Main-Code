package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class PinSubsystem extends SubsystemBase {
  private TalonSRX pinMotor = new TalonSRX(21);


// this is a comment


  public PinSubsystem() {
  }

  public void enablePin() {
    pinMotor.set(TalonSRXControlMode.PercentOutput, 1);
  }

  public void disablePin() {
    pinMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }
}