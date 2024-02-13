package universalSwerve.components.implementations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import universalSwerve.components.ITranslationSystem;
import universalSwerve.utilities.Conversions;

public class TalonSRXOpenLoopTranslationSystem implements ITranslationSystem{

    private TalonSRX mTalonSRX;
    private double mMotorFreeSpeedRPMs;
    private double mGearRatio;
    private double mWheelDiameter; 
    private double mLastSetVelocity =0;
    private double mLastSetPercentOutput= 0;

    public TalonSRXOpenLoopTranslationSystem(TalonSRX pTalonSRX, double pMotorFreeSpeedRPMs, double pGearRatio, double pWheelDiameter)
    {
            mTalonSRX = pTalonSRX;
            mMotorFreeSpeedRPMs = pMotorFreeSpeedRPMs;
            mGearRatio = pGearRatio;
            mWheelDiameter = pWheelDiameter;
    }

    @Override
    public void Initialize() {
     
        
    }

    @Override
    public void SetVelocity(double pVelocity) {
        //pVelocity is in inches per second
        //Let's convert that to RPMS
        //SmartDashboard.putNumber("VelocityInchesPerSecond", pVelocity);
        double rpms = Conversions.InchesPerSecondToRPMS(pVelocity, this.mWheelDiameter);        
        //SmartDashboard.putNumber("rpms", rpms);
        double rpmsAtMotor = rpms / mGearRatio;
        //SmartDashboard.putNumber("rpmsAtMotor", rpmsAtMotor);
        double percentOutput = rpmsAtMotor / mMotorFreeSpeedRPMs;
        mLastSetPercentOutput = percentOutput;

       mTalonSRX.set(TalonSRXControlMode.PercentOutput, percentOutput);
       mLastSetVelocity = pVelocity;
    }

    @Override
    public double GetVelocity() {
        return mLastSetVelocity;
    }

    @Override
    public double GetDistanceTravelled() {
        throw new RuntimeException("GetDistanceTravelled not supported for TalonSRXOpenLoopTranslationSystem");
    }

    @Override
    public void ResetDistanceTravelled() {
        throw new RuntimeException("ResetDistanceTravelled not supported for TalonSRXOpenLoopTranslationSystem");
    
        
    }

    @Override
    public void SetToBreakMode() {
       mTalonSRX.setNeutralMode(NeutralMode.Brake);
        
    }

    @Override
    public void SetToCoastMode() {
        mTalonSRX.setNeutralMode(NeutralMode.Coast);
        
    }

    @Override
    public void StopEverything() {
        mTalonSRX.set(ControlMode.PercentOutput, 0);
        
    }

    @Override
    public double GetPercentOutput() {
       return mLastSetPercentOutput;
    }
    
}
