package universalSwerve;

import universalSwerve.components.Wheel;
import universalSwerve.components.WheelLabel;
import universalSwerve.components.implementations.WheelMode;
import universalSwerve.controls.ISwerveControls;
import universalSwerve.hardware.IGyroscope;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.Conversions;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import odomoteryLogging.OdometryLogging_SwerveDriveOdometry;
import universalSwerve.utilities.SwerveNudgingDirection;

public class SwerveDrive 
{
    public double GetDistanceTravelled()
    {
        return mWheels[0].GetDistanceTravelled();
    }
  

    private Wheel mNEWheel;
    private Wheel mSEWheel;
    private Wheel mSWWheel;
    private Wheel mNWWheel;

    private Wheel[] mWheels;

    private IGyroscope mGyroscope;

    private SwerveDriveKinematics mSwerveDriveKinematics;

    private DrivingStyle mDrivingStyle = DrivingStyle.FIELD_RELATIVE;

    private double mMaxLinearSpeed; //meters per second, 2022 was 3.0
    private double mMaxRotationalSpeed; //degrees per second, 2022 was 120

    private double mLastIntentionalAngle = 0;
    private boolean mIsIntentionallyTurning = false;
    private double mNudgingSpeed = 0.2;

    private boolean mDiagnosticsEnabled = false;

    //private OdometryLogging_SwerveDriveOdometry mOdometry;
    private SwerveDriveOdometry mOdometry;

	public enum DrivingStyle
	{
		FIELD_RELATIVE,
		ROBOT_RELATIVE
	}

    private SwerveNudgingDirection mLastSwerveNudgingDirection = SwerveNudgingDirection.NONE;
    private long mTimeThatCurrentNudgingStarted = 0;

    public SwerveDrive(Wheel pNEWheel, Wheel pSEWheel, Wheel pSWWheel, Wheel pNWWheel, IGyroscope pGyroscope,
        double pMaxLinearSpeed, double pMaxRotationalSpeed, double pNudgingSpeed)
    {
        mNEWheel = pNEWheel;
        mSEWheel = pSEWheel;
        mSWWheel = pSWWheel;
        mNWWheel = pNWWheel;
        mWheels = new Wheel[4];
        mWheels[0] = mNEWheel;
        mWheels[1] = mSEWheel;
        mWheels[2] = mSWWheel;
        mWheels[3] = mNWWheel;

        mGyroscope = pGyroscope;

        mSwerveDriveKinematics = new SwerveDriveKinematics(
            mNEWheel.GetCenterToWheelTranslation(),
            mSEWheel.GetCenterToWheelTranslation(),
            mSWWheel.GetCenterToWheelTranslation(),
            mNWWheel.GetCenterToWheelTranslation());

        mMaxLinearSpeed = pMaxLinearSpeed;
        mMaxRotationalSpeed = pMaxRotationalSpeed;
        mNudgingSpeed = pNudgingSpeed;

        mAngleTrackController = new PIDController(0.1, 0, 0);
        mAngleTrackController.enableContinuousInput(0, 360);


        Initialize();


    }

    public void ConfigureDriveMotorsForGameMode(boolean pIsAutonomous)
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            mWheels[i].ConfigureDriveMotorsForGameMode(pIsAutonomous);
        }
    }


    public void InitializeOdometry()
    {
        if(mOdometry == null)
        {
        //Swerve Drive Odometry Code add for Choreo
        //Adapted from https://github.com/SleipnirGroup/ChoreoSwerveBot/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java#L111
        mOdometry = new SwerveDriveOdometry(
          mSwerveDriveKinematics,
          new Rotation2d(
                Math.toRadians(
                    AngleUtilities.ConvertOurAnglesToSwerveKinematicsAngles(mGyroscope.GetCurrentAngle()))), //hmmm has this been initialized  yet?  
            new SwerveModulePosition[] {
            mNEWheel.GetPosition(),
            mSEWheel.GetPosition(),
            mSWWheel.GetPosition(),
            mNWWheel.GetPosition()
          });
        }
    }
    
    //This should be called in each main loop iteration of the program.
    public void UpdateOdometry()
    {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
            mNEWheel.GetPosition(),
            mSEWheel.GetPosition(),
            mSWWheel.GetPosition(),
            mNWWheel.GetPosition()
            };

        /*
        
        SmartDashboard.putNumber("UpdateOdometry_NE_distance", swerveModulePositions[0].distanceMeters);
        SmartDashboard.putNumber("UpdateOdometry_NE_angle", swerveModulePositions[0].angle.getDegrees());

        SmartDashboard.putNumber("UpdateOdometry_SE_distance", swerveModulePositions[1].distanceMeters);
        SmartDashboard.putNumber("UpdateOdometry_SE_angle", swerveModulePositions[1].angle.getDegrees());

        SmartDashboard.putNumber("UpdateOdometry_SW_distance", swerveModulePositions[2].distanceMeters);
        SmartDashboard.putNumber("UpdateOdometry_SW_angle", swerveModulePositions[2].angle.getDegrees());

        SmartDashboard.putNumber("UpdateOdometry_NW_distance", swerveModulePositions[3].distanceMeters);
        SmartDashboard.putNumber("UpdateOdometry_NW_angle", swerveModulePositions[3].angle.getDegrees());
        */

        double gyroAngle = mGyroscope.GetCurrentAngle();
        double convertedAngle = AngleUtilities.ConvertOurAnglesToSwerveKinematicsAngles(gyroAngle);
        /*
        SmartDashboard.putNumber("UpdateOdometry_gyroAngle", gyroAngle);
        SmartDashboard.putNumber("UpdateOdometry_GyroInSwerveKinematics", convertedAngle);
        */

        mOdometry.update(
        new Rotation2d(
                Math.toRadians(convertedAngle)), //hmmm has this been initialized  yet? 
                    swerveModulePositions 
            );
        
        /*
        SmartDashboard.putNumber("Choreo_CurrentOdometry_X", mOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Choreo_CurrentOdometry_Y", mOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Choreo_CurrentOdometry_Rotation", mOdometry.getPoseMeters().getRotation().getDegrees());
        */
    }

    public void ResetOdometry(Pose2d pose)
    {
        mOdometry.resetPosition(
            new Rotation2d(
             Math.toRadians(
                    AngleUtilities.ConvertOurAnglesToSwerveKinematicsAngles(mGyroscope.GetCurrentAngle()))), 
            new SwerveModulePosition[] {
            mNEWheel.GetPosition(),
            mSEWheel.GetPosition(),
            mSWWheel.GetPosition(),
            mNWWheel.GetPosition()
            },
            pose);
    }
 
    public Pose2d getPose() 
    {
        return mOdometry.getPoseMeters();
    }

    private void Initialize()
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            mWheels[i].Initialize();
        }

        mGyroscope.SetCurrentAngle(0);
    }

   

    //ATS:  What's going on here:  In 2023 a white paper was written describing how
    //The swerve drive kinematics were not continious, leading to a slightly slewed
    //driving angle while turning
    //The code for 2024 WPILIB includes this change to account for it
    //Since this code is not in the publicly available code yet, I've cribbed it here
    private final boolean USE_DISCRETIZER= true;

    private static ChassisSpeeds discretize(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        double dtSeconds) {
      var desiredDeltaPose =
          new Pose2d(
              vxMetersPerSecond * dtSeconds,
              vyMetersPerSecond * dtSeconds,
              new Rotation2d(omegaRadiansPerSecond * dtSeconds));
      var twist = new Pose2d().log(desiredDeltaPose);
      return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
    }

    private ChassisSpeeds mLastRequestedChassisSpeeds;
    private SwerveModuleState[] CalculateModuleTargetStates(double linearSpeedFrontBackComponent, double linearSpeedLeftRightComponent, double requestedRotationalSpeed, double gyroAngle, DrivingStyle pDrivingStyle)
    {
        //First parameter is inches/s forward, second is inches/s to the left, third is radians/second counter clockwise
        //This works without gyro:
        //ChassisSpeeds requestedChassisSpeeds = 
        //	new ChassisSpeeds(linearSpeedFrontBackComponent, -1.0 * linearSpeedLeftRightComponent, -1.0 * Math.toRadians(requestedRotationalSpeed));
        //Now add in gyro:
        
        ChassisSpeeds requestedChassisSpeeds;
        

        double linearSpeedFrontBackComponent_MetersPerSecond = Conversions.InchesPerSecondToPetersPerSecond(linearSpeedFrontBackComponent);
        double linearSpeedLeftRightComponent_MetersPerSecond = Conversions.InchesPerSecondToPetersPerSecond(linearSpeedLeftRightComponent);

        
        if(pDrivingStyle == DrivingStyle.FIELD_RELATIVE)
        {
            requestedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearSpeedFrontBackComponent_MetersPerSecond, 
                    -1.0 * linearSpeedLeftRightComponent_MetersPerSecond, 
                    -1.0 * Math.toRadians(requestedRotationalSpeed),
                    Rotation2d.fromDegrees(AngleUtilities.ConvertOurAnglesToSwerveKinematicsAngles(gyroAngle)));
        }
        else //robot relative
        {
            requestedChassisSpeeds = new ChassisSpeeds(linearSpeedFrontBackComponent_MetersPerSecond, -1.0 * linearSpeedLeftRightComponent_MetersPerSecond, -1.0 * Math.toRadians(requestedRotationalSpeed));
        }								
        
    
        if(USE_DISCRETIZER)
        {
            mLastRequestedChassisSpeeds = discretize(requestedChassisSpeeds.vxMetersPerSecond, requestedChassisSpeeds.vyMetersPerSecond,
                requestedChassisSpeeds.omegaRadiansPerSecond , 0.005);

            return mSwerveDriveKinematics.toSwerveModuleStates( mLastRequestedChassisSpeeds); // the 0.02 is the loop timing (20ms) ...//changed to 0.005 based on our new setup timing to be 5ms
        }
        else
        {
            mLastRequestedChassisSpeeds = requestedChassisSpeeds;
            return mSwerveDriveKinematics.toSwerveModuleStates(requestedChassisSpeeds);
        }
    }

    


    private static double GetLinearSpeedLeftRightComponent(double pAngle, double pRequestedLinearSpeed)
	{
		//the angle here is our normal coordinates
		//0 is straight ahead, 90 is to the right, etc.
		//The speed is the total speed that we want, i.e., the hypotenuse of the velocity vector
		//this assumes positive speeds only.
		if(pAngle == 0)
		{
			return 0;
		}
		else if(pAngle < 90)
		{
			return pRequestedLinearSpeed * Math.sin( Math.toRadians(pAngle));
		}
		else if (pAngle == 90)
		{
			return pRequestedLinearSpeed;
		}
		else if (pAngle < 180)
		{
			return pRequestedLinearSpeed * Math.cos( Math.toRadians(pAngle-90.0));
		}
		else if(pAngle == 180)
		{
			return 0;
		}
		else if(pAngle < 270)
		{
			return -1.0 * pRequestedLinearSpeed * Math.sin(Math.toRadians(pAngle - 180.0));
		}
		else if(pAngle == 270)
		{
			return -1.0 * pRequestedLinearSpeed;
		}
		else if(pAngle < 360)
		{
			return -1.0 * pRequestedLinearSpeed * Math.cos(Math.toRadians(pAngle - 270.0));
		}
		else
		{
			return 0;//probably not a real case
		}
	}

	private static double GetLinearSpeedFrontBackComponent(double pAngle, double pRequestedLinearSpeed)
	{
		//the angle here is our normal coordinates
		//0 is straight ahead, 90 is to the right, etc.
		//The speed is the total speed that we want, i.e., the hypotenuse of the velocity vector
		//this assumes positive speeds only.
		if(pAngle == 0)
		{
			return pRequestedLinearSpeed;
		}
		else if(pAngle < 90)
		{
			return pRequestedLinearSpeed * Math.cos( Math.toRadians(pAngle));
		}
		else if (pAngle == 90)
		{
			return 0;
		}
		else if (pAngle < 180)
		{
			return -1.0* pRequestedLinearSpeed * Math.sin( Math.toRadians(pAngle-90.0));
		}
		else if(pAngle == 180)
		{
			return -pRequestedLinearSpeed;
		}
		else if(pAngle < 270)
		{
			return -1.0 * pRequestedLinearSpeed * Math.cos(Math.toRadians(pAngle - 180.0));
		}
		else if(pAngle == 270)
		{
			return 0;
		}
		else if(pAngle < 360)
		{
			return pRequestedLinearSpeed * Math.sin(Math.toRadians(pAngle - 270.0));
		}
		else
		{
			return 0;//probably not a real case
		}
	}


      /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * ATS:  This is copied straight from WPILib's SwerveModuleState class, but I don't like the fact that they use 90 here
   * In particular, if your wheel is spinning quickly, and you direct a 91 degree move, it would be stupid to 
   * chuck the motor into reverse just to save 2 degrees.  So we'll use an arbitrary 110 instead. 
   * We could come back to this and be a bit more clever and say "if the wheel is hardly spinning use 90 otherwise something a bit bigger" 
   * but that's a task for another day...
   * 
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   * @return Optimized swerve module state.
   */
    public static SwerveModuleState Optimize(SwerveModuleState desiredState, Rotation2d currentAngle) 
    {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 110.0) //ATS switched 90 to 110
        {
                return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } 
        else
        {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }


    private static SwerveModuleState OptimizeTargetState(SwerveModuleState pTargetState, double pCurrentWheelAngle)    
    {
        SwerveModuleState optimizedTargetState = Optimize(
                            pTargetState, 
                            Rotation2d.fromDegrees(AngleUtilities.ConvertOurAnglesToSwerveKinematicsAngles(pCurrentWheelAngle))
        );
        return optimizedTargetState;
    }

    private PIDController mAngleTrackController;
    /*
        Computes the module states for the given requested input
        pXTranslationComponent: [-1, 1]:  
            The percentage of the maximum translation speed to be given in the X direction.
            X here is Field relative when in Field relative mode or Robot Releative when in robot relative mode
            1 means "right" in the context above, -1 means "left in the context above
            This is generally mapped directly onto the left joystick X value when normal driving
        pYTranslationComponent: [-1, 1]:  
            The percentage of the maximum translation speed to be given in the Y direction.
            Y here is Field relative when in Field relative mode or Robot Releative when in robot relative mode
            1 means "forward" in the context above, -1 means "backwards" in the context above
            This is generally mapped directly onto the left joystick Y value when normal driving
        pTranslationVelocityPercentage: [0,1]:
            The percentage of the maximum linear speed for the module
        pRotationSpeedPercentage: [-1,1]:
            The percentage of the maximum rotational speed for the module
            +1 is clockwise, -1 is counte clockwise
        
    */
    private void CalculateAndApplyModuleStatesForSwerveInput(double pXTranslationComponent, double pYTranslationComponent, double pTranslationVelocityPercentage, double pRotationSpeedPercentage, DrivingStyle pDrivingStyle)
	{
		double controllerAngle = AngleUtilities.ConvertLinearComponentsToAngle(pXTranslationComponent, pYTranslationComponent);
		//SmartDashboard.putNumber("ControllerAngle", controllerAngle);
		double requestedLinearSpeed = pTranslationVelocityPercentage * mMaxLinearSpeed;

		double linearSpeedLeftRightComponent = GetLinearSpeedLeftRightComponent(controllerAngle, requestedLinearSpeed);		

		double linearSpeedFrontBackComponent = GetLinearSpeedFrontBackComponent(controllerAngle, requestedLinearSpeed);
		
		double requestedRotationalSpeed = pRotationSpeedPercentage * mMaxRotationalSpeed;
		
		// pass the swerver calculator each component and let it build (internally the target states)
		SwerveModuleState[] targetModelStates = CalculateModuleTargetStates(linearSpeedFrontBackComponent, linearSpeedLeftRightComponent, requestedRotationalSpeed, mGyroscope.GetCurrentAngle(), pDrivingStyle);
		// the states/targets have been calculated above, now assign
        //ATS Added for NY:
        //ATS 12/11/2023...  Why do we NOT do this??  Not sure why this is commented out.  Need to experiment.
        //ATS, added this back in on 3/4/2024, need to test
        targetModelStates = DesaturateWheelSpeeds(targetModelStates);
        for(int i = 0; i < mWheels.length; i++)
		{			
            //if(i==3)//just test NW
            {
                Wheel wheel = mWheels[i];
                
                
                SwerveModuleState targetState = targetModelStates[i];

                //this takes care of the "drive it in reverse if your are close to the right angle already" thing:
                SwerveModuleState optimizedTargetState = OptimizeTargetState(targetState, wheel.GetCurrentAngle());
                //SwerveModuleState optimizedTargetState =targetState;

                switch(wheel.GetWheelMode())
                {
                    case Enabled:
                        double targetAngle = AngleUtilities.ConvertSwerveKinematicsAnglesToOurAngles(optimizedTargetState.angle.getDegrees());                
                        wheel.SetWheelTargetAngle(targetAngle);
                        wheel.SetWheelVelocity(Conversions.MetersPerSecondToInchesPerSecond(optimizedTargetState.speedMetersPerSecond));
                        break;
                    case DontMove:
                        wheel.StopEverything();
                        break;
                    case Castor:
                        throw new RuntimeException("Castor not supported yet.");
                    default:
                        throw new RuntimeException("Unknown wheel mode.");
                        
                }
                /*
                if(i == 3)
                {
                    //log Nw
                    SmartDashboard.putNumber("CurrentWheelAngle", wheel.GetCurrentAngle());
                    SmartDashboard.putNumber("TargetAngle", targetAngle);
                }
                */

            }
		}		
	}

    private SwerveModuleState[] DesaturateWheelSpeeds(SwerveModuleState[] pTargetModelStates)
    {
        double absoluteMaxSpeed = 0;
        for(int i = 0; i < pTargetModelStates.length; i++)
        {
            absoluteMaxSpeed = Math.max(absoluteMaxSpeed, Conversions.MetersPerSecondToInchesPerSecond(Math.abs(pTargetModelStates[i].speedMetersPerSecond)));
        }
        double scale;
        if(absoluteMaxSpeed < this.mMaxLinearSpeed)
        {
            scale = 1;
        }
        else
        {
            scale =  this.mMaxLinearSpeed / absoluteMaxSpeed;
        }
        
        for(int i = 0; i < pTargetModelStates.length; i++)
        {
            pTargetModelStates[i].speedMetersPerSecond *= scale;
        }

        return pTargetModelStates;
    }

    public void StandardSwerveDrive(double pXTranslationComponent, double pYTranslationComponent, double pTranslationVelocityPercentage, double pRotationSpeedPercentage, boolean pTrackAngle, double pAngleToTrack)
    {

      
        if(Math.abs(pRotationSpeedPercentage) < 0.01)
        {
            this.mLastIntentionalAngle = mGyroscope.GetCurrentAngle();
            this.mIsIntentionallyTurning = false;
        }
        else
        {
            this.mIsIntentionallyTurning = true;
        }
        
   

        double computedRotationSpeedPercentage;
        if(pTrackAngle) 
        {
            mAngleTrackController.setSetpoint(pAngleToTrack);
            computedRotationSpeedPercentage = mAngleTrackController.calculate(mGyroscope.GetCurrentAngle());
            //ATS 3/4:
            //This can be rather extreme if you are way away from the target.  So let's limit it to be what you see at 90 off of setpoint degrees, currently.
            if(computedRotationSpeedPercentage < -90.0*0.1) //.1 is the P
            {
                computedRotationSpeedPercentage = -90.0*0.1;
            }
            else if(computedRotationSpeedPercentage > 90.0 * 0.1)
            {
                computedRotationSpeedPercentage = 90.0 * 0.1;
            }
        
        }
        else
        {
            computedRotationSpeedPercentage = pRotationSpeedPercentage;
        }

        CalculateAndApplyModuleStatesForSwerveInput(pXTranslationComponent, pYTranslationComponent, pTranslationVelocityPercentage, computedRotationSpeedPercentage, mDrivingStyle);

    }

    private double CalculateNudgeAccelerationMultiplier(long pStartedNudgingTimestamp, long pCurrentNudgingTimestamp)
    {
        //Let's do this...
        //for the first 1/10th of a second, don't accelerate at al.
        //For the next 9/10th of a second, scale linearly from original speed to double speed
        double returnValue;
        double TIME_BEFORE_ACCELERATION = 200;
        double TIME_TO_MAX_ACCELERATION = 1000;
        double MULTIPLIER_AT_MAX_ACCELERATION = 3.0;
        double DEFAULT_MULTIPLIER = 1.0;
        double timeSinceStarted = (double)pCurrentNudgingTimestamp - (double)pStartedNudgingTimestamp;
         SmartDashboard.putNumber("NudgetimeSinceStarted", timeSinceStarted);
        if(timeSinceStarted < TIME_BEFORE_ACCELERATION)
        {
            returnValue =  DEFAULT_MULTIPLIER;
        }
        else
        {
            double percentageIntoAccelerationWindow = Math.max(0, Math.min(1.0, (timeSinceStarted - TIME_BEFORE_ACCELERATION) / (TIME_TO_MAX_ACCELERATION - TIME_BEFORE_ACCELERATION)));
            returnValue = (percentageIntoAccelerationWindow * (MULTIPLIER_AT_MAX_ACCELERATION - DEFAULT_MULTIPLIER)) + DEFAULT_MULTIPLIER;            
        }
        SmartDashboard.putNumber("NudgeMultiplier", returnValue);
        return returnValue;
    }

    public void Nudge(SwerveNudgingDirection pDirection, double pNudgeSpeedMultiplier, double pAngleOffset)
    {
        if(pDirection != mLastSwerveNudgingDirection)
        {
            mTimeThatCurrentNudgingStarted = System.currentTimeMillis();
            mLastSwerveNudgingDirection = pDirection;
        }

        double xNudgingComponent = 0;
        double yNudgingComponent = 0;
        switch (pDirection) {
            case NORTH:
                yNudgingComponent = 1;
                xNudgingComponent = 0;
                break;
            case EAST:
                yNudgingComponent = 0;
                xNudgingComponent = 1;
                break;
            case SOUTH:
                yNudgingComponent = -1;
                xNudgingComponent = 0;
                break;
            case WEST:
                yNudgingComponent = 0;
                xNudgingComponent = -1;
                break;			
            default:
                yNudgingComponent = 0;
                xNudgingComponent = 0;
                break;
        }

        xNudgingComponent += pAngleOffset;


        this.mLastIntentionalAngle =  mGyroscope.GetCurrentAngle();

        double nudgeSpeed;
         
        if(AllWheelsAreClose())
        {
            nudgeSpeed = mNudgingSpeed;
        }
        else
        {
            nudgeSpeed = 0; //I checked the swerve kinematics and I don't think this 0.001 is necessary. 0.001;//we need something nonzero here so that the wheels will get an angle to turn to.
        } 

        nudgeSpeed *=  pNudgeSpeedMultiplier;
        nudgeSpeed *= CalculateNudgeAccelerationMultiplier(mTimeThatCurrentNudgingStarted, System.currentTimeMillis());

        CalculateAndApplyModuleStatesForSwerveInput(xNudgingComponent, yNudgingComponent, nudgeSpeed, 0, DrivingStyle.ROBOT_RELATIVE);
    
    }
   
    public void Nudge(SwerveNudgingDirection pDirection)
    {
        Nudge(pDirection, 1.0, 0.0);
    }

    public void TurnAllWheels(double pDegrees)
    {
        for(int i = 0; i < mWheels.length; i++)
		{			
            Wheel wheel = mWheels[i];
            double targetAngle = pDegrees;
                            
            wheel.SetWheelTargetAngle(targetAngle);
        }
    }

    private void SetWheelsToBreakMode()
    {
        for(int i = 0; i < mWheels.length; i++)
		{			
            Wheel wheel = mWheels[i];
            wheel.SetToBreakMode();             
        }
    }

    private void SetWheelsToCoastMode()
    {
        for(int i = 0; i < mWheels.length; i++)
		{			
            Wheel wheel = mWheels[i];
            
            wheel.SetToCoastMode();             

        }
    }


    
 
    private boolean AllWheelsAreClose()
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            //SmartDashboard.putBoolean(mWheels[i].GetWheelLabel().Text() + "_IsClose", mWheels[i].IsCloseToTargetAngle());
            if(!mWheels[i].IsCloseToTargetAngle())
            {
                return false;
            }
        }
        return true;
    }

    public void StopTranslationButAllowWheelDirection()
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            mWheels[i].StopTranslationButAllowWheelDirection();
        }
    }


    public void StopEverything()
    {
        mLastSwerveNudgingDirection = SwerveNudgingDirection.NONE;
        for(int i = 0; i < mWheels.length; i++)
        {
            mWheels[i].StopEverything();
        }
    }

      /*
        Computes and sets module states for the given requested input
        pXTranslation: meters per second
            The percentage of the maximum translation speed to be given in the X direction.
            X here is Field relative when in Field relative mode or Robot Releative when in robot relative mode
            1 means "right" in the context above, -1 means "left in the context above
            This is generally mapped directly onto the left joystick X value when normal driving
        pYTranslationComponent: meters per second
            The percentage of the maximum translation speed to be given in the Y direction.
            Y here is Field relative when in Field relative mode or Robot Releative when in robot relative mode
            1 means "forward" in the context above, -1 means "backwards" in the context above
            This is generally mapped directly onto the left joystick Y value when normal driving
        pRotationSpeed: Maybe Radians per second?
            ???
        
    */


    //This method is intended to be called by Choreo
    public void Run(double pXTranslationMetersPerSecond, double pYTranslationMetersPerSecond, double pRotationSpeed )
    {
        mLastSwerveNudgingDirection = SwerveNudgingDirection.NONE;
        /*
        SmartDashboard.putNumber("ChoreoCommand_timeStamp", System.currentTimeMillis());
        SmartDashboard.putNumber("Choreo_pXTranslationMetersPerSecond", pXTranslationMetersPerSecond);
        SmartDashboard.putNumber("Choreo_pYTranslationMetersPerSecond", pYTranslationMetersPerSecond);
        SmartDashboard.putNumber("Choreo_pRotationSpeed", pRotationSpeed);
        */
         SwerveModuleState[] targetModelStates = CalculateModuleTargetStates(
            Conversions.MetersPerSecondToInchesPerSecond(pYTranslationMetersPerSecond),
            Conversions.MetersPerSecondToInchesPerSecond(pXTranslationMetersPerSecond), 
            -1.0 *Math.toDegrees(pRotationSpeed), //sWERVE kINEMATICS turns in opposite direction from our units, so flip it here
             0, DrivingStyle.ROBOT_RELATIVE);
         for(int i = 0; i < mWheels.length; i++)
		{			
            {
                Wheel wheel = mWheels[i];                            
                SwerveModuleState targetState = targetModelStates[i];
                //this takes care of the "drive it in reverse if your are close to the right angle already" thing:
                SwerveModuleState optimizedTargetState = OptimizeTargetState(targetState, wheel.GetCurrentAngle());
                
                //SmartDashboard.putNumber("Choreo_NE_X_DesiredSpeed", optimizedTargetState.speedMetersPerSecond);

                switch(wheel.GetWheelMode())
                {
                    case Enabled:
                        double targetAngle = AngleUtilities.ConvertSwerveKinematicsAnglesToOurAngles(optimizedTargetState.angle.getDegrees());                
                        wheel.SetWheelTargetAngle(targetAngle);
                        wheel.SetWheelVelocity(Conversions.MetersPerSecondToInchesPerSecond(optimizedTargetState.speedMetersPerSecond));
                        break;
                    case DontMove:
                        wheel.StopEverything();
                        break;
                    case Castor:
                        throw new RuntimeException("Castor not supported yet.");
                    default:
                        throw new RuntimeException("Unknown wheel mode.");
                        
                }               
            }
		}
    }

    public void Run(ISwerveControls pControls)
    {
        //SmartDashboard.putNumber("Old Gyro Angle", mGyroscope.GetCurrentAngle());
        SwerveNudgingDirection nudgingDirection = pControls.GetSwerveNudgingDirection();
        if(nudgingDirection == SwerveNudgingDirection.NONE)
        {
            mLastSwerveNudgingDirection = SwerveNudgingDirection.NONE;

            double rotation = pControls.GetSwerveRotationalSpeed();
            if(Math.abs(rotation) < 0.10)
            {
                rotation = 0;
            }

            StandardSwerveDrive(pControls.GetSwerveXComponent(), pControls.GetSwerveYComponent(), pControls.GetSwerveLinearSpeed(), rotation, pControls.GetTrackSpecificAngle(), pControls.GetSpecificAngleToTrack());
        }
        else
        {
            if(nudgingDirection == SwerveNudgingDirection.EAST)
            {
                Nudge(SwerveNudgingDirection.EAST);
            }
            else if(nudgingDirection == SwerveNudgingDirection.WEST)
            {
                Nudge(SwerveNudgingDirection.WEST);
            }
            else if(nudgingDirection == SwerveNudgingDirection.NORTH)
            {
                Nudge(SwerveNudgingDirection.NORTH);
            }
            else// (nudgingDirection == Controls.SwerveNudgingDirection.SOUTH)
            {
                Nudge(SwerveNudgingDirection.SOUTH);
            }
        }
    }

    public void Run(ISwerveControls pControls, boolean pShouldTrackSpecificAngle, double pAngleToTrack)
    {
        SwerveNudgingDirection nudgingDirection = pControls.GetSwerveNudgingDirection();
        if(nudgingDirection == SwerveNudgingDirection.NONE)
        {
            mLastSwerveNudgingDirection = SwerveNudgingDirection.NONE;

            double rotation = pControls.GetSwerveRotationalSpeed();
            if(Math.abs(rotation) < 0.10)
            {
                rotation = 0;
            }

            StandardSwerveDrive(pControls.GetSwerveXComponent(), pControls.GetSwerveYComponent(), pControls.GetSwerveLinearSpeed(), rotation, pShouldTrackSpecificAngle, pAngleToTrack);
        }
        else
        {
            if(nudgingDirection == SwerveNudgingDirection.EAST)
            {
                Nudge(SwerveNudgingDirection.EAST);
            }
            else if(nudgingDirection == SwerveNudgingDirection.WEST)
            {
                Nudge(SwerveNudgingDirection.WEST);
            }
            else if(nudgingDirection == SwerveNudgingDirection.NORTH)
            {
                Nudge(SwerveNudgingDirection.NORTH);
            }
            else// (nudgingDirection == Controls.SwerveNudgingDirection.SOUTH)
            {
                Nudge(SwerveNudgingDirection.SOUTH);
            }
        }
    }

    public void SetGyroscopeCurrentAngle(double pCurrentAngle)
    {
        mGyroscope.SetCurrentAngle(pCurrentAngle);
    }

    public double GetGyroscopeCurrentAngle()
    {
        return mGyroscope.GetCurrentAngle();
    }

    public void ResetDistanceTravelled()
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            mWheels[i].ResetDistanceTravelled();
        }
    }

    /*
    public void LogDriveData()
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            SmartDashboard.putNumber(mWheels[i].GetWheelLabel().Text() + "_Velocity", mWheels[i].GetVelocity());
            SmartDashboard.putNumber(mWheels[i].GetWheelLabel().Text() + "_PercentOutput", mWheels[i].GetPercentOutput());
            
            
        }
    }
    */

    private Wheel GetWheelByLabel(WheelLabel pLabel)
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            if(mWheels[i].GetWheelLabel().equals(pLabel))
            {
                return mWheels[i];
            }
        }
        throw new RuntimeException("Unknown wheel label:" + pLabel.Text());
    }

    public ChassisSpeeds getLastRequestedChassisSpeeds()
    {
        return mLastRequestedChassisSpeeds;
    }

    

    public void EnableDiagnostics(WheelLabel pLabel)
    {
        GetWheelByLabel(pLabel).EnableDiagnostics();
    }

    public void DisableDiagnostics(WheelLabel pLabel)
    {
        GetWheelByLabel(pLabel).DisableDiagnostics();
    }

    public void SetWheelMode(WheelLabel pLabel, WheelMode pWheelMode)
    {
        GetWheelByLabel(pLabel).SetWheelMode(pWheelMode);
    }

    public void EnableDiagnostics()
    {
        mDiagnosticsEnabled = true;
    }

    public void DisableDiagnostics()
    {
        mDiagnosticsEnabled = false;    
    }
    public void LogDiagnostics()
    {
        if(mDiagnosticsEnabled)
        {
            for(int i = 0; i < mWheels.length; i++)
            {
                mWheels[i].LogDiagnostics();
            }


            SmartDashboard.putNumber("GyroAngle",mGyroscope.GetCurrentAngle());
        }

    }

    public java.util.List<TalonFX> GetSpeakers()
    {
        java.util.ArrayList<TalonFX> returnValue = new ArrayList<TalonFX>();
        for(int i = 0; i < mWheels.length; i++)
        {
            returnValue.addAll(mWheels[i].GetSpeakers());
        }
        return returnValue;
    }



}
