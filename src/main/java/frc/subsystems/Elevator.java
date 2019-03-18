package frc.subsystems;
import frc.robot.Robot;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends PIDSubsystem {
    public static Encoder liftEncoder;
	public Spark elevatorSpark = new Spark(2);
 	//public static Encoder liftEncoder = Encoder(0, 1, 2, EncodingType.k4X);
    public DigitalInput top = new DigitalInput(0);
    public DigitalInput bottom = new DigitalInput(1);
    
	static double p = 0.01;
	static double i = 0.0;
	static double d = 0.0;
	double height;
    int level = 0;
    int setMotorDirection;
    boolean override = false;
    
//    public double[] potVolts = {
//    		0, .167, .334, .501, .635, .881, 1
//    };
    
    public double[] setPoints = {
    		 0,
      		(12.1 / 12),
      		(24.2 / 12),
      		(36.3 / 12),
      		(48.4 / 12),
      		(60.5 / 12),
      		(72.6 / 12)
    };
    
    public int[] feet = {
    		0, 1, 2, 3, 4, 5, 6
    };

    public Elevator() {
    	super("Elevator", p, i, d);
        setAbsoluteTolerance(0.005);     
        getPIDController().setContinuous(true);
        if (Robot.isReal()) {
        	liftEncoder.setDistancePerPulse(.00000579);
	    } else {
	    	liftEncoder.setDistancePerPulse((4.0/*in*/*Math.PI)/(360.0*12.0/*in/ft*/));
	    }
        liftEncoder.setDistancePerPulse(0.01);
	    // 0.01060287527213352263064593786011 inches per pulse
	    // 94.314039761290039374540986558176  pulses per inch
        // .000005794527654503616 corrected number
	    // 0.12723450326560227156775125432132 feet per pulse
	    // 1131.7684771354804724944918386981 pulses per foot
    }
 
    public void initDefaultCommand() {
   // 	setDefaultCommand(new ElevateWithTriggers());
   // 	liftEncoder.setPIDSourceParameter(PIDSourceParameter.kDistance);
    }
    
    public void switchOverride () {
    	override = !override;
    	SmartDashboard.putBoolean("Override?", override);
    }
    
    public void elevate(double input) {
	    
    	double height = liftEncoder.getDistance();
    	Command command;
    	if(height > 26.2){
    //cmd here
    	} else if(height > 19.3) {
	//cmd here	
    	} else if(height > 11.33) {
    //cmd here
    	} else if(height > 0.01){
    //cmd here
    	} else {
    //cmd here
    	}
    	
    	
		// Take the raw input if no switches are engaged.
    	if (override == true) {		
	    	elevatorSpark.set(input);
			return;
	    } else if (bottom.get() == true && input < 0) {
	//    	Robot.elevator.getLiftEncoder().reset();  
			stop();
			return;
		} else {
			elevatorSpark.set(input);
		}
    }
    
    public void stop() {
    	elevatorSpark.set(0);
    }
    
    //---------------------------------------//
    
    /**
     * Use the potentiometer as the PID sensor. This method is automatically
     * called by the subsystem.
     */
    protected double returnPIDInput() {
    	height = liftEncoder.getDistance();
    	return height;
    }
    
    /**
     * Use the motor as the PID output. This method is automatically called by
     * the subsystem.
     */
    protected void usePIDOutput(double output) {
        // Prevents lift from smacking the floor.
    	if (bottom.get() == true && output < 0) {
    //		Robot.elevator.getLiftEncoder().reset();        	
    		stop();
    		return;
        } else {
        	elevatorSpark.set(output * setMotorDirection);
        	return;
        }
    }
    
    // Set maximum
    public void incLevel() {
    	setMotorDirection = 1;
    	this.setSetpoint(12.1);
    }
    
    // Set minimum
    public void decLevel() {
    	setMotorDirection = -1;
    	this.setSetpoint(12.1);
    }
    
    public void setLevel(int level) {
    	this.setSetpoint(setPoints[level]);
    }
    
    //---------------------------------------//
    
    public Encoder getLiftEncoder() {
    	return liftEncoder;
    }
    
    public void log() {
    	SmartDashboard.putData("Raw Lift Encoder", liftEncoder);
    	SmartDashboard.putNumber("Lift Distance", liftEncoder.getDistance());
    	SmartDashboard.putBoolean("At Bottom?", bottom.get());
    }
}