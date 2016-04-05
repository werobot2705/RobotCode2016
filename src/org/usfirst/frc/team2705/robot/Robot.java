
package org.usfirst.frc.team2705.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;


public class Robot extends SampleRobot implements PIDOutput {
	RobotDrive myDrive, myDrive2;
	Joystick stick;
	CANTalon left, right, roller, left2,right2;
	Talon hookA, hookB, upper, rollerArm, arm1, arm2;	
	Timer timer;
	Shooter shooter;
	DigitalInput ballHeldSwitch;
	double time;
	double timeEnd; 
    public int ballHeld = 0;
    public double rollerArmSpeed = 0.5;
    
    //Extra gyro stuff done by other team members. Ignore
    Gyro gyro;
	static final double Kp = 0.03;
	//
	
	public boolean[] autonomous = {false, false, false, false, false};
	public int dashAutoBool = 0;
	CameraServer camera;
	
	public int turnToAngleTime = 3000;
	public long startTime;
	
	public final int A_BUTTON = 1;
	public final int Y_BUTTON = 4;
	public final int RIGHT_STICK_PRESS = 10;
	public final int LEFT_STICK_PRESS = 9;
	public final int LEFT_BUMPER = 5;
	public final int RIGHT_BUMPER = 6;
	public final int RIGHT_GOAL_ANGLE = 55;
	public final int LEFT_GOAL_ANGLE = 120;
	
	// Gyro stuff?
	private int angle;
	private boolean turnToAngle = false;
	private PIDController turnController;
	private static double kP = 0.03;
	private static final double kI = 0.00;
	private static final double kD = 0.00;
	private static final double kF = 0.00;
	
	private static final double kToleranceDegree = 1.0f;
	
	public void robotInit(){
		arm1 = new Talon(0);
		arm2 = new Talon(2);
		left = new CANTalon(2);
		right = new CANTalon(1);
		left2 = new CANTalon(4);
		right2 = new CANTalon(6);
		myDrive = new RobotDrive(left, right);
		myDrive2 = new RobotDrive(left2,right2);
		stick = new Joystick(1);	//Left stick
		shooter = new Shooter(999);
		ballHeldSwitch = new DigitalInput(0);
		camera = CameraServer.getInstance();
		camera.setQuality(20);
		camera.startAutomaticCapture("cam0");
		// Buttons: A = 1; B = 2; X = 3; Y = 4; LB = 5; RB = 6; sel = 7; Start = 8; LT = 9; RT = 10
		
		AHRS ahrs = new AHRS(SPI.Port.kMXP);
		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		   turnController.setInputRange(-180.0f, 180.0f);
		   turnController.setOutputRange(-1.0, 1.0);
		   turnController.setAbsoluteTolerance(kToleranceDegree);
		   turnController.setContinuous(true);
	}

public void autonomous(){
    myDrive.setSafetyEnabled(false);
    myDrive2.setSafetyEnabled(false);
    
	String autoMode = "moat";
	
	if(autoMode == "0"){//drive to defense
       myDrive.arcadeDrive(-.5, 0.0);
       myDrive2.arcadeDrive(-.5, 0.0);
        System.out.println("Autonomous A is running!");
        Timer.delay(5.0);
        myDrive.arcadeDrive(0.0, 0.0);
        myDrive2.arcadeDrive(0.0, 0.0);
        
    }else if(autoMode == "1"){//portcullis
    	arm1.set(-1);
		arm2.set(-1);
		Timer.delay(0.7);
		arm1.set(0);
		arm2.set(0);
	    myDrive.arcadeDrive(-0.65, 0.0);
	    myDrive2.arcadeDrive(-0.65, 0.0);
	    Timer.delay(3.0);
	    myDrive.arcadeDrive(-0.4 , 0.0);
	    myDrive2.arcadeDrive(-0.4, 0.0);
	    arm1.set(1);
		arm2.set(1);
		Timer.delay(0.75);
		arm1.set(0);
		arm2.set(0);
		myDrive.arcadeDrive(-0.65, 0.0);
		myDrive2.arcadeDrive(-0.65, 0.0);
		Timer.delay(3.0);
		
	    
    }else if(autoMode == "trump"){//drive past defense - low bar, drive-overs
	    myDrive.arcadeDrive(-0.75, 0.0);
	    myDrive2.arcadeDrive(-0.75, 0.0);
	    Timer.delay(5.0); //4 for rock wall
	    myDrive.arcadeDrive(0.0, 0.0);
	    myDrive2.arcadeDrive(0.0, 0.0);
	    
       }else if(autoMode == "moat"){//drive past defense - low bar, drive-overs
    	   myDrive.arcadeDrive(-80.0, 0.0);
    	   myDrive2.arcadeDrive(-80.0, 0.0);
    	   Timer.delay(3.0); //4 for rock wall
    	   myDrive.arcadeDrive(0.0, 0.0);
    	   myDrive2.arcadeDrive(0.0, 0.0);
       }
   }
   
   public void operatorControl(){
	   myDrive.setSafetyEnabled(true);
	   myDrive2.setSafetyEnabled(true);
	   while (isOperatorControl() && isEnabled()){
		   //initialize the arcade drive
		   myDrive.arcadeDrive(stick);
		   myDrive2.arcadeDrive(stick);
		   //Slight delay to make sure we don't send commands to fast
		   Timer.delay(0.01);
		   
		   // more gyro stuff?
		   
		   
		   if(stick.getRawButton(LEFT_BUMPER)){
			   angle = LEFT_GOAL_ANGLE;
			   turnToAngle = true;
			   startTime = System.currentTimeMillis();
		   } else if(stick.getRawButton(RIGHT_BUMPER)){
			   angle = RIGHT_GOAL_ANGLE;
			   turnToAngle = true;
			   startTime = System.currentTimeMillis();
		   } else if (System.currentTimeMillis() - startTime > turnToAngleTime){
			   turnToAngle = false;
		   }
		   
		   if(turnToAngle){
			   if(!turnController.onTarget()){
				   turnController.setSetpoint(angle);
				   turnController.enable();
			   } else {
				   turnController.disable();
				   turnToAngle = false;
			   } 
		   } else {
			   turnController.disable();
		   }
 
		   SmartDashboard.putBoolean("Ball Held Switch", ballHeldSwitch.get());
		   
		   shooter.RunShooterStateMachine(stick.getRawButton(RIGHT_STICK_PRESS),
				   ballHeldSwitch.get(), stick.getRawButton(LEFT_STICK_PRESS)); // R & L Stick

		   if (stick.getRawButton(A_BUTTON)){ // d
			   arm1.set(-1);
			   arm2.set(-1);
		   } else if(stick.getRawButton(Y_BUTTON)){ // u
			   arm1.set(1);
			   arm2.set(1);
		   } else {
			   arm1.set(0);
			   arm2.set(0);
		   }
			   
	   }
   	}

@Override
public void pidWrite(double output) {
	// TODO Auto-generated method stub

	   myDrive.arcadeDrive(turnController.get(), -turnController.get());
	   myDrive2.arcadeDrive(turnController.get(), -turnController.get());
	
}
}
