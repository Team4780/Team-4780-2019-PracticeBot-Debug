package frc.commands.lift;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class  ElevateWithTriggers extends Command {

    public ElevateWithTriggers() {
        requires(elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	elevator.disable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Joystick joystick2;
    	double throttle = joystick2.getThrottle();
    	elevator.elevate(throttle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    protected void end() {
    	elevator.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	elevator.stop();
    }
}
