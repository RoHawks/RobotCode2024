package robotcode.autonomous.steps;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.AllianceInfo;
import robosystems.Intake;
import robotcode.autonomous.AAction;
import universalSwerve.SwerveDrive;

public class FollowTrajectoryAction extends AAction {

    private ChoreoTrajectory mTrajectory;
    private SwerveDrive mSwerveDrive;
    private boolean mIsFirstTrajectoryMovement;
    private Intake mIntake;
    private Boolean mShouldBeSuckingInRings;

    public boolean ShouldMirrorTrajectories() //To do - read this from driver station?
    {
        return AllianceInfo.GetInstance().ShouldFlipAutos();
    }
    
  private Command mChoreoCommand;
  private void CreateChoreoCommand()
  {
      var thetaController = new PIDController(10, 0, 0);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      mChoreoCommand = Choreo.choreoSwerveCommand(
        mTrajectory, // Choreo trajectory from above
        mSwerveDrive::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(0.8, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(0.8, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error

        
        /* this is what the choreo documentation says, but I think we think about things quite different from them in terms of X/Y
        (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false),
        */
        (ChassisSpeeds speeds) -> mSwerveDrive.Run( // needs to be robot-relative
          -speeds.vyMetersPerSecond,    
          speeds.vxMetersPerSecond,            
          speeds.omegaRadiansPerSecond),
        this::ShouldMirrorTrajectories ,       
        new Subsystem[0]
        //, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        //I suspect because I am calling the Run command above I don't need to spcity this? mSwerveDrive // The subsystem(s) to require, typically your drive subsystem only
    );
  }



    public FollowTrajectoryAction(String pTrajectoryName, SwerveDrive pSwerveDrive, boolean pIsFirstTrajectoryMovement, Intake pIntake, Boolean pShouldBeSuckingInRings)
    {
        mTrajectory = Choreo.getTrajectory(pTrajectoryName);
        mSwerveDrive = pSwerveDrive;
        mIsFirstTrajectoryMovement = pIsFirstTrajectoryMovement;
        mIntake = pIntake;
        mShouldBeSuckingInRings = pShouldBeSuckingInRings;
        CreateChoreoCommand();
    }

    public void EnterAction()
    {
        super.EnterAction();
        if(mIsFirstTrajectoryMovement)
        {
            //mSwerveDrive.ResetOdometry(mTrajectory.getInitialPose());
            mSwerveDrive.ResetOdometry(mTrajectory.sample(-1, ShouldMirrorTrajectories()).getPose());
        }
        mChoreoCommand.initialize();

    }

    @Override
    public boolean Run() {
        mSwerveDrive.UpdateOdometry();
        //This just checks if the timing of the trajectory is complete
        //Should generally work
        //But may not if someone bumps into us, becuase it will end before we got to where we want to be.
        mChoreoCommand.execute();
        
        if (mShouldBeSuckingInRings != null)
        {
          SmartDashboard.putBoolean("mShouldBeSuckingInRings",mShouldBeSuckingInRings);
          if(mShouldBeSuckingInRings)
          {
            mIntake.setToInstaLaunch();
          }
          else
          {
            mIntake.setToHoldingSpeed();
          }
        }
          
        return mChoreoCommand.isFinished();
    }

    @Override
    public void EndAction() {
        //Perhaps don't do anything.
    }

    
}
