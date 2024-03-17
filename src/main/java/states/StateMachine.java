package states;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightManager;
import robosystems.*;
import universalSwerve.SwerveDrive;
import universalSwerve.SwerveFactory;

public class StateMachine 
{
    private HashMap<States, AState> mStates;
    private States mCurrentState = null;
    

    
    public StateMachine(Controls pControls, SwerveDrive pSwerveDrive, Intake pIntake, Shooter pShooter, ExtendoArm pExtendoArm, ClimberArms pClimberArms, Lights pLights, LimelightManager pLimelightManager)
    {     
        mStates = new HashMap<States, AState>();

        IntakingState intakingState = new IntakingState(pSwerveDrive, pIntake, pShooter, pExtendoArm, pControls, pLights);
        mStates.put(intakingState.GetState(), intakingState);

        HoldingState holdingState = new HoldingState(pSwerveDrive, pIntake, pShooter, pExtendoArm, pControls, pLights, pLimelightManager);
        mStates.put(holdingState.GetState(), holdingState);

        ShootingState shootingState = new ShootingState(pSwerveDrive, pIntake, pShooter, pExtendoArm, pControls, pLights, pLimelightManager);
        mStates.put(shootingState.GetState(), shootingState);

        ClimbingState climbingState = new ClimbingState(pSwerveDrive, pClimberArms, pShooter, pIntake, pControls);
        mStates.put(climbingState.GetState(), climbingState);

        ClimbingPreparationState climbingPreparationState = new ClimbingPreparationState(pSwerveDrive, pClimberArms, pShooter, pControls, pIntake);
        mStates.put(climbingPreparationState.GetState(), climbingPreparationState);

        TrapShootingState trapShootingState = new TrapShootingState(pControls);
        mStates.put(trapShootingState.GetState(), trapShootingState);

        EjectingState ejectingState = new EjectingState(pSwerveDrive, pIntake, pShooter, pExtendoArm, pControls, pLights);
        mStates.put(ejectingState.GetState(), ejectingState);
        
        mCurrentState = intakingState.GetState();
    }


     public void TransitionBackFromManualMode()
     {
        if (mCurrentState != null)
        {
            AState currentStateObject = mStates.get(mCurrentState);
            currentStateObject.ExitState();
        }
        mCurrentState = States.Intaking;
        AState newCurrentStateObject = mStates.get(mCurrentState);
        newCurrentStateObject.EnterState(null);   
     }

    public void Reset()
    {
      
        //Call this at the beginning of teleop and auto
        //Moves us to GameStart state (which may do nothing if it's already been done)
        if(mCurrentState != null)
        {
            AState currentStateObject = mStates.get(mCurrentState);
            currentStateObject.ExitState();
            
            mCurrentState = States.Intaking;
            AState newCurrentStateObject = mStates.get(mCurrentState);
            newCurrentStateObject.EnterState(ShooterMode.Undefined);
        }
    }

    public void Run()
    {
        


        AState currentStateObject = mStates.get(mCurrentState);
        //SmartDashboard.putString("CurrentState", currentStateObject.GetName());
        NextStateInfo nextStateInfo = currentStateObject.Run();
        if(nextStateInfo.GetNextState() != mCurrentState)
        {            
            currentStateObject.ExitState();
            
            //SmartDashboard.putString("Next State Info", (nextStateInfo.GetNextState().toString()));

            AState newState = mStates.get(nextStateInfo.GetNextState());
            /*
            mLog.append((System.currentTimeMillis() - mStartTime) / 1000);
            mLog.append(": ");
            mLog.append(newState.GetName());
            SmartDashboard.putString("StateMachineLog", mLog.toString());
            */
            newState.EnterState(nextStateInfo.GetNextStateParameter());
            mCurrentState = nextStateInfo.GetNextState();
        }        
    }

    public void log()
    {
        mStates.get(mCurrentState).log();
    }
    
}
