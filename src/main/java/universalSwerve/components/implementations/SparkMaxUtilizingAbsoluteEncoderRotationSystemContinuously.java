package universalSwerve.components.implementations;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.components.IRotationSystem;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.PIDFConfiguration;

public class SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously implements IRotationSystem
{
    private CANSparkMax mSparkMax;    
	private SparkMaxAbsoluteEncoder mAbsoluteEncoder;

	
    public SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously(CANSparkMax pSparkMax,  PIDFConfiguration pPidfConfiguration)
    {
        mSparkMax = pSparkMax;
		mAbsoluteEncoder = mSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		SetPIDParameters(pPidfConfiguration);        

    }

    private void SetPIDParameters(PIDFConfiguration pPidfConfiguration)
    {

        
		mSparkMax.getPIDController().setFF(pPidfConfiguration.F());
		mSparkMax.getPIDController().setP(pPidfConfiguration.P());        
		mSparkMax.getPIDController().setI(pPidfConfiguration.I());        
		mSparkMax.getPIDController().setD(pPidfConfiguration.D());		        
		mSparkMax.getPIDController().setOutputRange(-1, 1);        
        mSparkMax.getPIDController().setPositionPIDWrappingEnabled(true);        
        mSparkMax.getPIDController().setPositionPIDWrappingMaxInput(1);
        mSparkMax.getPIDController().setPositionPIDWrappingMinInput(0);               
		
    }

    @Override
    public void Initialize() 
    {
		//Pretty sure nothing needs to be done 
    }

	
    @Override
    public double GetCurrentAngle() {
        
        return AngleUtilities.Normalize(ConvertEncoderUnitToOurAngle(mAbsoluteEncoder.getPosition()));
    }
	@Override
    public double GetRawCurrentAngle() {
        
        return GetCurrentAngle();
    }


    private double ConvertOurAngleToEncoderUnit(double pAngle)
    {
        return (360.0 - AngleUtilities.Normalize(pAngle)) / 360.0;
    }

    private double ConvertEncoderUnitToOurAngle(double pEncoderUnit)
    {
        return AngleUtilities.Normalize((1.0 - pEncoderUnit) * 360.0);
    }

    @Override
    public void SetAngle(double pTargetAngle) {
        double encoderReference = ConvertOurAngleToEncoderUnit(pTargetAngle);
        //SmartDashboard.putNumber("encoderReference", encoderReference);
        mSparkMax.getPIDController().setReference(encoderReference, CANSparkMax.ControlType.kPosition);
        
        
    }
    

    /**
	Returns the number of full rotations of the turn motor
	*/
	public double GetAngle()
	{
		return ConvertEncoderUnitToOurAngle(mSparkMax.getEncoder().getPosition());
	}

    @Override
    public void StopEverything() {
        mSparkMax.set(0);
        
    }

	



}
