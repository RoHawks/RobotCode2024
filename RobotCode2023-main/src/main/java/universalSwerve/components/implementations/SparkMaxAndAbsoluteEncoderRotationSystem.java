package universalSwerve.components.implementations;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.components.IRotationSystem;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.PIDFConfiguration;

public abstract class SparkMaxAndAbsoluteEncoderRotationSystem implements IRotationSystem
{
    private CANSparkMax mSparkMax;
    private double mInitialAngle;
    private double mGearRatioBetweenMotorAndWheel;


	/*
	The gear ratio passed in here should be a fraction, so if it's 100:1, pass in 0.01
	*/
    public SparkMaxAndAbsoluteEncoderRotationSystem(CANSparkMax pSparkMax, double pGearRatioBetweenMotorAndWheel, PIDFConfiguration pPidfConfiguration)
    {
        mSparkMax = pSparkMax;
		SetSparkMaxParameters(pPidfConfiguration);
        mGearRatioBetweenMotorAndWheel = pGearRatioBetweenMotorAndWheel;

    }

    private void SetSparkMaxParameters(PIDFConfiguration pPidfConfiguration)
    {
		mSparkMax.restoreFactoryDefaults();
        mSparkMax.setInverted(true);
		mSparkMax.setOpenLoopRampRate(0.35);
		mSparkMax.setSmartCurrentLimit(20);		
		mSparkMax.getPIDController().setFeedbackDevice(mSparkMax.getEncoder());
		mSparkMax.getEncoder().setPosition(0);
		mSparkMax.getPIDController().setFF(pPidfConfiguration.F());
		mSparkMax.getPIDController().setP(pPidfConfiguration.P());
		mSparkMax.getPIDController().setI(pPidfConfiguration.I());
		mSparkMax.getPIDController().setD(pPidfConfiguration.D());
		
		mSparkMax.getPIDController().setOutputRange(-1, 1);
		
    }

    @Override
    public void Initialize() 
    {
        mInitialAngle = GetCurrentAngleFromAbsoluteEncoder();    
		mSparkMax.getEncoder().setPosition(0);
    }

	protected abstract double GetCurrentAngleFromAbsoluteEncoder();

    @Override
    public double GetCurrentAngle() {
        
        return GetWheelCurrentAngleFromNeoPlusAbsoluteEncoder();
    }

    @Override
    public void SetAngle(double pTargetAngle) {
		SmartDashboard.putNumber("WheelTargetAngle", pTargetAngle);
		double motorTargetPosition = CalculateEncoderTargetForWheelTarget(pTargetAngle);
		SmartDashboard.putNumber("MotorTargetPosition", motorTargetPosition);
        mSparkMax.getPIDController().setReference(motorTargetPosition, CANSparkMax.ControlType.kPosition);
        
    }
    

    /* 
	Returns the number of full rotations of the turn motor
	*/
	private double  GetTurnMotorRotationsSinceStartUp()
	{
		return mSparkMax.getEncoder().getPosition();
	}

	/*
		Returns the amount in degrees represented by the ticks passed in.  In Neo550 space, a "tick" is one rotation of the motor. 
	*/
    private double ConvertMotorRotationsToDegrees(double pMotorTicks)
	{
		double ans = (pMotorTicks * mGearRatioBetweenMotorAndWheel * 360.0);
        return AngleUtilities.Normalize(ans);
	}


	/*
		Takes the initial measurement from the absolute encoder and adds the relative motion of the motor since then.
	*/
	private double GetWheelCurrentAngleFromNeoPlusAbsoluteEncoder()
	{
		return AngleUtilities.Normalize(
                mInitialAngle + 
				ConvertMotorRotationsToDegrees(GetTurnMotorRotationsSinceStartUp())
			);
			
	}

	public abstract double GetRawCurrentAngle();
	/*
		This figures out the target rotations ("position") for the Neo550 to acheive the pTargetAngle of the wheel.
	*/
    private double CalculateEncoderTargetForWheelTarget(double pTargetAngle)
	{
		SmartDashboard.putNumber("GetWheelCurrentAngleFromNeoPlusAbsoluteEncoder", GetWheelCurrentAngleFromNeoPlusAbsoluteEncoder());
		SmartDashboard.putNumber("GetTurnMotorRotationsSinceStartUp", GetTurnMotorRotationsSinceStartUp());

        return  CalculateWheelTargetEncoderPositionNeo(pTargetAngle, GetWheelCurrentAngleFromNeoPlusAbsoluteEncoder(), GetTurnMotorRotationsSinceStartUp());
    }

	private double CalculateWheelTargetEncoderPositionNeo(double pTargetAngle, double pCurrentAngle, double pCurrentNeoPosition)
	{
		// From Adam's spreadsheet
		//
		// Wheel target has three inputs:
		//   - the target angle from the controller
		//   - the current wheel angle according to the lamprey
		//   - the current turn motor position
		//
		// returns the target position that the wheel should turn to

		//B14:
		double amountToMoveWheelToGetToDesiredAngleDegrees = pTargetAngle - pCurrentAngle; //-32
		
		if(amountToMoveWheelToGetToDesiredAngleDegrees > 180.0)
		{
			amountToMoveWheelToGetToDesiredAngleDegrees -= 360.0;
		}
		else if(amountToMoveWheelToGetToDesiredAngleDegrees < -180.0)
		{
			amountToMoveWheelToGetToDesiredAngleDegrees += 360.0;
		}
		SmartDashboard.putNumber("amountToMoveWheelToGetToDesiredAngleDegrees", amountToMoveWheelToGetToDesiredAngleDegrees);
		//B15:
		double amountToMoveMotorToGetToDesiredAngleEncoderPositions = amountToMoveWheelToGetToDesiredAngleDegrees / 360.0  / mGearRatioBetweenMotorAndWheel;
		SmartDashboard.putNumber("amountToMoveMotorToGetToDesiredAngleEncoderPositions", amountToMoveMotorToGetToDesiredAngleEncoderPositions);
		//B16:
		
		double targetEncoderPosition = pCurrentNeoPosition + amountToMoveMotorToGetToDesiredAngleEncoderPositions;
		SmartDashboard.putNumber("targetEncoderPosition", targetEncoderPosition);
		return targetEncoderPosition;
	}

	@Override
    public void StopEverything() {
        mSparkMax.set(0);
        
    }
}
