// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import java.util.function.Supplier;

public class ManualMoveArmCommand extends CommandBase {
  private final Arm arm;
  private final Supplier<Double> speedSupplier;

  /** Creates a new ManualMoveArmCommand. */
  public ManualMoveArmCommand(Arm arm, Supplier<Double> speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.speedSupplier = speedSupplier;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setMotorSpeed(speedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
