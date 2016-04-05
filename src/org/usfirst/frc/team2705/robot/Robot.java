
package org.usfirst.frc.team2705.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	public final int LEFT_GOAL_ANGLE = -55;
	
	private double autoSpeed = -.75;
	// Gyro stuff?
	private int angle;
	private boolean turnToAngle = false;
	private PIDController turnController;
	private static double kP = 0.03;
	private static final double kI = 0.00;
	private static final double kD = 0.00;
	private static final double kF = 0.00;
	
	private static final double kToleranceDegree = 1.0f;
	
	 SendableChooser chooser;
	 public static SendableChooser obstacleSelector;
	
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
		   
		obstacleSelector = new SendableChooser();
		   
		obstacleSelector.addDefault("Default Obstacle", 2);
	    obstacleSelector.addObject("Moat Obstacle", 3);
	    obstacleSelector.addObject("Portcullis Obstacle", 4);
	    obstacleSelector.addObject("Low Bar Obstacle", 5);
	        
	    SmartDashboard.putData("Obstacle Selector", obstacleSelector);
	}

public void autonomous(){
    myDrive.setSafetyEnabled(false);
    myDrive2.setSafetyEnabled(false);
    
	String autoMode = "moat";
	
	int autoSelect = (int) obstacleSelector.getSelected();
	
	if(autoMode == "0"){//drive to defense
       myDrive.arcadeDrive(-.5, 0.0);
       myDrive2.arcadeDrive(-.5, 0.0);
        System.out.println("Autonomous A is running!");
        Timer.delay(5.0);
        myDrive.arcadeDrive(0.0, 0.0);
        myDrive2.arcadeDrive(0.0, 0.0);
        
    }else if(autoSelect == 4){//portcullis
    	autoSpeed = -.65;
    	
    	turnController.setSetpoint(0);
		turnController.enable();
    	arm1.set(-1);
		arm2.set(-1);
		Timer.delay(0.7);
		arm1.set(0);
		arm2.set(0); 
		   long start = System.currentTimeMillis();
		   while (System.currentTimeMillis() - start < 3000) {
		   	myDrive.arcadeDrive(autoSpeed, turnController.get());
		   	myDrive2.arcadeDrive(autoSpeed, turnController.get());
		   }
		   start = System.currentTimeMillis();
		   while (System.currentTimeMillis() - start < 750) {
		   	myDrive.arcadeDrive(-.4, turnController.get());
		   	myDrive2.arcadeDrive(-.4, turnController.get());
		   	arm1.set(1);
		   	arm2.set(1);
		   }
		   arm1.set(0);
		   arm2.set(0);
		   start = System.currentTimeMillis();
		   while (System.currentTimeMillis() - start < 2000) {
		   	myDrive.arcadeDrive(autoSpeed, turnController.get());
		   	myDrive2.arcadeDrive(autoSpeed, turnController.get());
		   }
		    myDrive.arcadeDrive(0.0, 0.0);
		    myDrive2.arcadeDrive(0.0, 0.0);
//	    myDrive.arcadeDrive(-0.65, 0.0);
//	    myDrive2.arcadeDrive(-0.65, 0.0);
//	    Timer.delay(3.0);
//	    myDrive.arcadeDrive(-0.4 , 0.0);
//	    myDrive2.arcadeDrive(-0.4, 0.0);
//	    arm1.set(1);
//		arm2.set(1);
//		Timer.delay(0.75);
//		arm1.set(0);
//		arm2.set(0);
//		myDrive.arcadeDrive(-0.65, 0.0);
//		myDrive2.arcadeDrive(-0.65, 0.0);
//		Timer.delay(3.0);
		
	    
    }else if(autoSelect == 2){//default
	   autoSpeed = -.75;
	   turnController.setSetpoint(0);
	   turnController.enable();
	   long start = System.currentTimeMillis();
	   while (System.currentTimeMillis() - start < 4500) {
	   	myDrive.arcadeDrive(autoSpeed, turnController.get());
	   	myDrive2.arcadeDrive(autoSpeed, turnController.get());
	   }
	    //myDrive.arcadeDrive(-0.75, 0.0);
	    //myDrive2.arcadeDrive(-0.75, 0.0);
	    //Timer.delay(5.0); //4 for rock wall
	    myDrive.arcadeDrive(0.0, 0.0);
	    myDrive2.arcadeDrive(0.0, 0.0);
	    turnController.disable();
	    
       }else if(autoSelect == 3){//moat
    	   autoSpeed = -.8;
	   long start = System.currentTimeMillis();
	    turnController.setSetpoint(0);
	   turnController.enable();
	   while (System.currentTimeMillis() - start < 2500) {
	   	myDrive.arcadeDrive(autoSpeed, turnController.get());
	   	myDrive2.arcadeDrive(autoSpeed, turnController.get());
	   }
    	   //myDrive.arcadeDrive(-80.0, 0.0);
    	   //myDrive2.arcadeDrive(-80.0, 0.0);
    	   //Timer.delay(3.0); //4 for rock wall
    	   myDrive.arcadeDrive(0.0, 0.0);
    	   myDrive2.arcadeDrive(0.0, 0.0);
    	   turnController.disable();
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
	if (isOperatorControl()) {
	   myDrive.tankDrive(output, -output);
	   myDrive2.tankDrive(output, -output);
	}
}
}
