/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.Gamepad;
import frc.commands.lift.*;
import frc.subsystems.Elevator;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
// auto choices in smartdashbord/shuffleboard  
  private static final String kAutoLine = "Drive Straight - Auto Line";
  private static final String kAutoLineRight = "Drive Straight - Turn Right";
  private static final String kAutoLineLeft = "Drive Straight - Turn Left";
//


// essential instatiation
  public static Compressor compressor;
  public static DoubleSolenoid hatchGrabberSolenoid;
  public static DoubleSolenoid hatchGrabberDeploySolenoid;
  public static PowerDistributionPanel pdp;
//

// encoder instantiation 3/2/19
private static final int kEncoderPortA = 0;
private static final int kEncoderPortB = 1;
private Encoder m_encoder;
// --end

// smartdashboard auto choices dropdown
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
// --end


//port instantiation
private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
private static final int leftVictorPort= 0;
private static final int rightVictorPort = 1;
private static final int kJoystickPort = 0;
private static final int kJoystick2Port = 1;
private static final int kElevatorPort = 2;
private static final int kCargoIntakePort = 3;
private static final int kHabClimbPort = 4;
// --end


// new gyro instantiation 
int P, I, D = 1;
private static final double kP = 0.005; // propotional turning constant
double angle;
boolean turned = true;
int mustTurnDegree = 0;
// --end


//int integral, previous_error, setpoint = 0;
  int integral, previous_error = 0;
  int setpoint = 20; //attempting to set the setpoint as a different number to see if it turns to that angle or not
// --end


// drive victorsp's
  VictorSP leftVictorSP = new VictorSP(leftVictorPort);
  VictorSP rightVictorSP = new VictorSP(rightVictorPort);
// --end


//auxillary sparks
Spark elevatorSpark = new Spark(kElevatorPort); //single - one controller
Jaguar cargoIntakeSPX = new Jaguar(kCargoIntakePort); //double - two controllers - one signal
//VictorSPX habClimbJaguar = new VictorSPX(kHabClimbPort); //double - two controllers - one signal
// --end


//hall effect
DigitalInput hallEffectSensorTop = new DigitalInput(0); //top sensor init
DigitalInput hallEffectSensorBottom = new DigitalInput(1); //bottom sensor init
DigitalInput topPreLimit = new DigitalInput(2); //top prelimit
DigitalInput bottomPreLimit = new DigitalInput(3); //bottom prelimit
DigitalInput rocketLowLevel = new DigitalInput(4); //rocket low level sensor init
DigitalInput rocketMiddleLevel = new DigitalInput(5); //rocket middle level sensor init
DigitalInput rocketHighLevel = new DigitalInput(6); //rocket high level sensor init
DigitalInput cargoShipLevel = new DigitalInput(7); //cargo ship level sensor init
// --end


// drivetrain (with victorsp's)
DifferentialDrive m_myRobot = new DifferentialDrive(leftVictorSP, rightVictorSP);
	private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);
  private Joystick m_joystick = new Joystick(kJoystickPort);
// --end


//second joystick instantiation
private Joystick m_joystick2 = new Joystick(kJoystick2Port);
// --end


  @Override
  public void robotInit() {
  compressor = new Compressor(1);
  hatchGrabberSolenoid = new DoubleSolenoid(4, 5);
  hatchGrabberDeploySolenoid = new DoubleSolenoid(6, 7);
  m_joystick2 = new Joystick(1);
  m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
  m_encoder.setDistancePerPulse((Math.PI * 1.8) / 360.0);

  // smartdashboard option adding  
    m_chooser.setDefaultOption("Drive Straight - Auto Line", kAutoLine);
    m_chooser.addOption("Drive Straight - Turn Right", kAutoLineRight);
    m_chooser.addOption("Drive Straight - Turn Left", kAutoLineLeft);
    SmartDashboard.putData("Auto choices", m_chooser);
  //   --end


  // gyro calibrate at robot turn on  
    m_gyro.calibrate();
  // --end


  // camera instatiation
  CameraServer camera = CameraServer.getInstance();
  VideoSource usbCam = camera.startAutomaticCapture("cam0", 0);
  usbCam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
  // --end
  }


  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  // unsure whether functioning but supposed to put angle of gyro in smartdashboard
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
  // --end
}

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Drive Straight - Auto Line", kAutoLine);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kAutoLine:
      default:
      /* NEW gyro angle setpoint testing (not working as of 2/2/19, doesnt move anywhere)
      m_myRobot.setSafetyEnabled(false);
      double error = setpoint - m_gyro.getAngle(); // Error = Target - Actual
      angle = m_gyro.getAngle() % 360;
      if(angle-20 > 0)m_myRobot.arcadeDrive(0.8, (angle - error)*kP);
      else if(angle+20 < 0)m_myRobot.arcadeDrive(0.8, (angle + error)*kP);
     // else m_myRobot.arcadeDrive(m_joystick.getY(), m_joystick.getX());    
      //m_myRobot.arcadeDrive(0, 0);
      --end */
      break;
      case kAutoLineRight:
      //code goes here
            //DO NOT USE UNTIL FIXED -- DRIVES FORWARDS AT FULL SPEED INFINITELY -- USE ONLY ON ROBOT CART AND BE EXTRA CAUTIOUS
            //I commented out this code to make sure that it can't be selected accidently. - Kyle 2/28/19
            /*
            double error2 = setpoint - m_gyro.getAngle(); // Error = Target - Actual
            this.integral += (error2*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
            double derivative = (error2 - this.previous_error) / .02;
            double finalangle = P * error2 + I * this.integral + D * derivative;
            m_myRobot.arcadeDrive(0, finalangle);

            */

        break;
      case kAutoLineLeft:
      //code goes here  
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  public void turnDegrees(int degree) {
    if(turned)return;
    angle = m_gyro.getAngle() % 360;
    if(angle-10 > degree)m_myRobot.arcadeDrive(0.8, (angle - degree)*kP);
    else if(angle+10 < degree)m_myRobot.arcadeDrive(0.8, (angle + degree)*kP);
    else turned = true;
    }

  @Override
  public void teleopPeriodic() {
  //Clear ALL Sticky Faults from PDP and PCM 
  //  pdp.clearStickyFaults();
    compressor.clearAllPCMStickyFaults();
  // --end  

  boolean upTriggered = hallEffectSensorTop.get() == false;
  boolean downTriggered = hallEffectSensorBottom.get() == false;

/* commented out on 2/28/19 for button elevator with prelimit stops
  //hall effect for elevator (tested & working as of 1/26/19)
	  double joystickYAxis = m_joystick2.getY();    
   
    if (joystickYAxis > 0 && upTriggered || joystickYAxis < 0 && downTriggered)
      {
        elevatorSpark.set(0); 
        
      }
    else
      {
      	elevatorSpark.set(m_joystick2.getY());
      }
    // --end
*/

//Pneumatics (tested & working as of 2/2/19)
//compressor.setClosedLoopControl(false);

//if-else piston control statement (tested & working as of 2/2/19)
  if (m_joystick2.getRawButton(4)) {
    hatchGrabberSolenoid.set(DoubleSolenoid.Value.kForward);
  } 
  else{  
  }	

  if (m_joystick2.getRawButton(3)) {
    hatchGrabberSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  else{
  }
// --end


//hatch grabber deploy outside of frame perimeter (written on 2/9/19, working as of N/A)
  if (m_joystick2.getRawButton(9)) {
    hatchGrabberDeploySolenoid.set(DoubleSolenoid.Value.kForward);
    System.out.println("Hatch Grabber Deployed");
  } 
  else{  
  }	
  
  if (m_joystick2.getRawButton(10)) {
    hatchGrabberDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
    System.out.println("Hatch Grabber NOT Deployed");
  }
  else{
  }
// --end


// pressure switch boolean's (tested & working as of 1/26/19)
boolean pressureSwitchOn = compressor.getPressureSwitchValue() == true;
boolean pressureSwitchOff = compressor.getPressureSwitchValue() == false;
// --end

/*
// compressor pressure regulation (tested & working as of 1/26/19)
  if(pressureSwitchOff) {
    compressor.enabled();
  }
  else if (pressureSwitchOn) {
    compressor.setClosedLoopControl(true);
  }
// --end
*/

//NEW (and working 100%) Gyro Math (tested & working as of 2/9/19)
m_myRobot.arcadeDrive(m_joystick.getY()*0.8, m_joystick.getX()*0.8);

if(m_joystick.getRawButton(1))turned = true;
if(m_joystick.getPOV() != -1){
  turned = false;
  mustTurnDegree = m_joystick.getPOV();
}
if(!turned)turnDegrees(mustTurnDegree);
// --end


//Basic Auxillary Functions (X for in and A for out) (written on 2/9/19, working as of N/A)
/* commented out 2/19/19 for SPX .set command not working correctly
if (m_joystick.getRawButton(1)) {
 cargoIntakeSPX.set(ControlMode PercentageOutput, 0.5, 0.5);  
}
else if (m_joystick.getRawButton(2)){
  cargoIntakeSPX.set(-0.5);
}
*/

//Elevator Hall Levels (Right Side Bumper and Trigger for Up and Down) (written on 2/9/19, working as of N/A)
boolean lowTriggered = rocketLowLevel.get() == false;
boolean middleTriggered = rocketMiddleLevel.get() == false;
boolean highTriggered = rocketHighLevel.get() == false;
boolean cargoShipTriggered = cargoShipLevel.get() == false;
// --end 

/* 2/19/19
//Buttons to control elevator (written on 2/9/19, working as of N/A)}
if (m_joystick2.getRawButton(6)) {
  elevatorSpark.set(0.5);
}
else if(m_joystick2.getRawButton(8)){
  elevatorSpark.set(-0.5);
}

if (m_joystick2.getRawButton(6) && upTriggered || m_joystick2.getRawButton(8) && downTriggered)
{
  elevatorSpark.set(0);
}
// --end
*/
/* 2/19/19
//Rocket and Cargo Ship level stops (written on 2/9/19, working as of N/A)
if (m_joystick2.getRawButton(8) || m_joystick.getRawButton(6)) {}
else if (m_joystick2.getRawButton(8) &&lowTriggered){ //req for cargo level
e levatorSpark.set(0);
}
else if (m_joystick2.getRawButton(8) &&middleTriggered){
elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(8) &&highTriggered){
elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(8) &&cargoShipTriggered){
elevatorSpark.set(0);
}  
else if (m_joystick2.getRawButton(6) &&lowTriggered){ //req for cargo level
elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(6) &&middleTriggered){
elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(6) &&highTriggered){
elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(6) &&cargoShipTriggered){
elevatorSpark.set(0);
} */ 
// --end


boolean topPreLimitTriggered = topPreLimit.get() == false;
boolean bottomPreLimitTriggered = bottomPreLimit.get() == false;
boolean button8NotPressed = !m_joystick2.getRawButton(8);
boolean button6NotPressed = !m_joystick2.getRawButton(6);

double definedSpeedDown = 0.5;
double definedSpeedUp = -0.5;


if (m_joystick2.getRawButton(6)) {
  elevatorSpark.set(definedSpeedUp);
}
else if (m_joystick2.getRawButton(8)) {
  elevatorSpark.set(definedSpeedDown);
}
else if (button6NotPressed || button8NotPressed){
  elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(6) && topPreLimitTriggered) {
definedSpeedUp = 0.2;
}
else if (m_joystick2.getRawButton(8) && bottomPreLimitTriggered) {
definedSpeedDown = -0.2;
}
else if (m_joystick.getRawButton(6) && upTriggered) {
elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(8) && downTriggered) {
elevatorSpark.set(0);
}
else if (m_joystick2.getRawButton(6) && bottomPreLimitTriggered) {
definedSpeedDown = -0.5;
}
else if (m_joystick2.getRawButton(8) && topPreLimitTriggered) {
definedSpeedUp = 0.5;
}
}

/**
   * This function is called periodically during test mode.
   */  
  @Override
  public void testPeriodic() {
  }
}