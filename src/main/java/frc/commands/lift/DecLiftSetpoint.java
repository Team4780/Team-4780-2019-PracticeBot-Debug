package frc.commands.lift;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DecLiftSetpoint extends Command {

	public DecLiftSetpoint() {
	   	requires(Robot.elevator);
	}
	
    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.elevator.enable();
        Robot.elevator.decLevel();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.elevator.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	 Robot.elevator.disable();
    	Robot.elevator.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevator.disable();
    	Robot.elevator.stop();
    }
}

