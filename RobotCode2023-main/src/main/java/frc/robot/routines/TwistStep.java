package frc.robot.routines;

import java.util.List;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TwistStep extends AComponentStep 
{

    private CANSparkMax mWristMotor;
    private Encoder mWristEncoder;
    private PIDController mWristPidController;
    private double mTargetAngle;

    public double WRIST_TICKS_PER_MOTOR_REVOLUTION = 28.0;
    public double WRIST_GEAR_RATIO = (68.0 / 13.0) * (68.0 / 13.0) * (68.0 / 13.0) ;

    public double mPassPositiveBound;
    public double mPassNegativeBound;

    public double mMaxVelocity;
    private boolean mReturnHome;
    private boolean mSkipThisStep;

    public TwistStep(String pName, List<StepPrequisite> pPrerequisites, CANSparkMax pWristMotor, Encoder pWristEncoder, double pPassPositiveBound, double pPassNegativeBound, double pMaxVelocity, boolean pReturnHome, boolean pSkipThisStep)
    {
        super(pName, pPrerequisites);
        mWristEncoder = pWristEncoder;
        mWristMotor = pWristMotor;
        mWristPidController = new PIDController(0.025, 0, 0);
        mTargetAngle = 0;
        mPassPositiveBound = pPassPositiveBound;
        mPassNegativeBound = pPassNegativeBound;
        mMaxVelocity = pMaxVelocity;
        mReturnHome = pReturnHome;
        mSkipThisStep = pSkipThisStep;
    }

    public double ConvertWristEncoderToDegrees(double pEncoderTicks)
    {
      //deg = 360.0 * pEncoderTicks / (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION); 
      return 360.0 * pEncoderTicks / (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION);
    }
  
    public double ConvertDegreesToWristEncoderTicks(double pDegrees)
    {
      //ticks = (deg / 360) * (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION); 
      return (pDegrees / 360.0) * (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION);
    }

    private double GetLocalTargetAngle()
    {
        return -1.0 * (mReturnHome ? 0 : mTargetAngle);
    }    

    @Override
    public void Run() {
        if(!mSkipThisStep)
        {
            double localTargetAngle = GetLocalTargetAngle();
            mWristPidController.setSetpoint(localTargetAngle);
            double wristPower = mWristPidController.calculate(ConvertWristEncoderToDegrees(mWristEncoder.getRaw()));
            wristPower = Math.max(-mMaxVelocity, Math.min(mMaxVelocity, wristPower));
            mWristMotor.set(wristPower);
        }
        
    }

    @Override
    public double DistanceFromCompletion() {
        if(mSkipThisStep)
        {
            return 0;
        }
        else
        {
            double localTargetAngle = GetLocalTargetAngle();
            return ConvertWristEncoderToDegrees(mWristEncoder.getRaw()) - localTargetAngle;
        }
    }

    @Override
    public boolean IsComplete() {
        if(mHasBeenCompleted)
        {
            return true;
        }
        if(mSkipThisStep)
        {
            return true;
        }
        double error = DistanceFromCompletion();
        //SmartDashboard.putNumber("WristError", error);
        mHasBeenCompleted = mHasBeenStarted &&  error < mPassPositiveBound && error > mPassNegativeBound;
        return mHasBeenCompleted;
    }

    
    public void Reset(double pClawAngle)
    {
        super.Reset(pClawAngle);
        mTargetAngle = pClawAngle;

    }
    
    @Override
    public String GetLogInfo() {
        return String.format("%.3f",DistanceFromCompletion());
    }
}
