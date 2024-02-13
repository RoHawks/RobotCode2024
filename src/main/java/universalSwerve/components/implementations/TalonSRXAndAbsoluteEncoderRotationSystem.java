package universalSwerve.components.implementations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.components.IRotationSystem;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.Conversions;
import universalSwerve.utilities.PIDFConfiguration;

public class TalonSRXAndAbsoluteEncoderRotationSystem  implements IRotationSystem{
    

    private TalonSRX mTalonSRX;
    private double mSensorReadingAtStraightAhead;
    private boolean mSensorIncreasesInOppositeDirectionOfWheelRotation;

    public TalonSRXAndAbsoluteEncoderRotationSystem(TalonSRX pTalonSRX, PIDFConfiguration pPidfConfiguration, double pSensorReadingAtStraightAhead, boolean pSensorIncreasesInOppositeDirectionOfWheelRotation)
    {
        mTalonSRX = pTalonSRX;
        mSensorReadingAtStraightAhead = pSensorReadingAtStraightAhead;
		SetTalonSRXParameters(pPidfConfiguration);
        mSensorIncreasesInOppositeDirectionOfWheelRotation = pSensorIncreasesInOppositeDirectionOfWheelRotation; 
       

    }

    private void SetTalonSRXParameters(PIDFConfiguration pPidfConfiguration)
    {

        mTalonSRX.setInverted(false);
        mTalonSRX.setSensorPhase(true);
		mTalonSRX.config_kP(0, pPidfConfiguration.P());
        mTalonSRX.config_kI(0, pPidfConfiguration.I());
        mTalonSRX.config_kD(0, pPidfConfiguration.D());
        
		
		
    }

    @Override
    public void Initialize() 
    {
        
    }

	protected double GetCurrentAngleFromAbsoluteEncoder()
    {
        double val = Conversions.CTREMagneticAbsoluteEncoderToDegrees(mTalonSRX.getSelectedSensorPosition(), mSensorReadingAtStraightAhead);
        if(mSensorIncreasesInOppositeDirectionOfWheelRotation)
        {
            return AngleUtilities.ChangeClockWiseiness(val);
        }
        else
        {
            return val;
        }
    }

    @Override
    public double GetCurrentAngle() {
        
        return GetCurrentAngleFromAbsoluteEncoder();
    }

    @Override
    public void SetAngle(double pTargetAngle) {
		//SmartDashboard.putNumber("WheelTargetAngle", pTargetAngle);
		double motorTargetPosition = CalculateEncoderTargetForWheelTarget(pTargetAngle);
		//SmartDashboard.putNumber("MotorTargetPosition", motorTargetPosition);
        mTalonSRX.set(ControlMode.Position, motorTargetPosition);
        
    }

    private double GetRawEncoderTicks()
    {
        return mTalonSRX.getSelectedSensorPosition();
    }
	/*
		This figures out the target rotations ("position") for the Neo550 to acheive the pTargetAngle of the wheel.
	*/
    private double CalculateEncoderTargetForWheelTarget(double pTargetAngle)
	{
        return  CalculateWheelTargetEncoderPositionNeo(pTargetAngle, GetCurrentAngle(), GetRawEncoderTicks());
    }

	private double CalculateWheelTargetEncoderPositionNeo(double pTargetAngle, double pCurrentAngle, double pCurrentEncoderTicks)
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
        SmartDashboard.putNumber("InHereCurrentAngle", pCurrentAngle);

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
		double amountToMoveMotorToGetToDesiredAngleEncoderPositions = amountToMoveWheelToGetToDesiredAngleDegrees / 360.0  * 4096.0;
		SmartDashboard.putNumber("amountToMoveMotorToGetToDesiredAngleEncoderPositions", amountToMoveMotorToGetToDesiredAngleEncoderPositions);
		//B16:
		
		double targetEncoderPosition ;
        if(mSensorIncreasesInOppositeDirectionOfWheelRotation)
        {
            targetEncoderPosition = pCurrentEncoderTicks - amountToMoveMotorToGetToDesiredAngleEncoderPositions;
        }
        else
        {
            targetEncoderPosition = pCurrentEncoderTicks + amountToMoveMotorToGetToDesiredAngleEncoderPositions;
        }
		SmartDashboard.putNumber("targetEncoderPosition", targetEncoderPosition);
		return targetEncoderPosition;
	}

	@Override
    public void StopEverything() {
        mTalonSRX.set(ControlMode.PercentOutput, 0);
        
    }

    @Override
    public double GetRawCurrentAngle() {
        return mTalonSRX.getSelectedSensorPosition();
    }
}
