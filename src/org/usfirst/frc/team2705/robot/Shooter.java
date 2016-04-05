package org.usfirst.frc.team2705.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

public class Shooter {
	public enum ShooterState {
		WaitForBall,
		BallHeld,
		AboveSwitch,
		DownToSwitch,
		PrepToShoot,
		RepositionBall,
		ReadyToShoot,
		Shoot
	};
	
	Shooter(int WTF){
		ShooterInit();
	}
	
	private org.usfirst.frc.team2705.robot.Shooter.ShooterState shooterState; //declare our variable which tracks the state
	private long StartTime;
	private CANTalon shooter, roller;
	
	/*************************************************************************************************
	 *                                   Parameters to tune
	 ************************************************************************************************/
	
	private int shooterSpinUp = 3000;
	private int shootTime = 1000;
	private int safeTime = 500;
	
	private void SetShooter(ShooterState shooterState) {
		this.shooterState = shooterState;
	}
	private ShooterState getShooterState(){
		return shooterState;
	}
	
	/************************************************************************************************
	 *                                    Public functions
	 ***********************************************************************************************/
	
	public void ShooterInit(){
		SetShooter(ShooterState.DownToSwitch);
		shooter = new CANTalon(3);
		roller = new CANTalon(0);
	}
	
	public void ShooterReset(){
		SetShooter(ShooterState.WaitForBall);
		shooter.set(0);
		roller.set(1);
	}
	
	public void RunShooterStateMachine(boolean buttonPress, boolean ballHeld, boolean cancel){
	
		if(cancel){
			SetShooter(ShooterState.WaitForBall);
		}
		
		switch (getShooterState()){
		case WaitForBall:
			shooter.set(0);
			roller.set(1);
			if(ballHeld){
				SetShooter(ShooterState.AboveSwitch);
			}
			else
			{
			roller.set(1);
			}
			break;
		case AboveSwitch:
			roller.set(1);
			if(!ballHeld){
				shooter.set(1);
				Timer.delay(0.03);
				shooter.set(0);
				roller.set(0);
				SetShooter(ShooterState.DownToSwitch);
			}
			break;
		case DownToSwitch:
			if(buttonPress){
				roller.set(-1);
				//SetShooter(ShooterState.BallHeld);
			}
			if(ballHeld){
				SetShooter(ShooterState.PrepToShoot);
			}
			break;
		case BallHeld:
			if (ballHeld){
				SetShooter(ShooterState.PrepToShoot);
			}
			break;
		case PrepToShoot:
			if (!ballHeld){
				roller.set(1);
				shooter.set(.8);	
				//StartTime = System.currentTimeMillis();
				SetShooter(ShooterState.RepositionBall);
			
			}
			break;
		case RepositionBall:
			if (ballHeld){
				roller.set(0);
				StartTime = System.currentTimeMillis();
				SetShooter(ShooterState.ReadyToShoot);
			}
			break;
		case ReadyToShoot:
			if (System.currentTimeMillis() - StartTime > shooterSpinUp){
				if (buttonPress){
					roller.set(1);
					StartTime = System.currentTimeMillis();
					SetShooter(ShooterState.Shoot);
				}
			}
			break;
		case Shoot:
			if (System.currentTimeMillis() - StartTime > shootTime){
				SetShooter(ShooterState.WaitForBall);
			}
			break;
		default:
		}
		
	}

}
