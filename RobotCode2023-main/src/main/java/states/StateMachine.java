package states;

import java.util.HashMap;



import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robosystems.*;
import universalSwerve.SwerveDrive;
import universalSwerve.SwerveFactory;

public class StateMachine 
{
    private HashMap<States, AState> mStates;
    private States mCurrentState = null;
    
    public StateMachine(Controls pControls, SwerveDrive pSwerveDrive, Intake pIntake, Claw pClaw, Wrist pWrist, Arm pArm, Wall pWall)
    {     
        mStates = new HashMap<States, AState>();

        
        GameStartState gameStartState = new GameStartState(pSwerveDrive, pIntake, pArm);
        mStates.put(gameStartState.GetState(), gameStartState);

        IntakingState intakingState = new IntakingState(pSwerveDrive, pIntake, pArm, pControls, pWall);
        mStates.put(intakingState.GetState(), intakingState);

        ScoringState scoringState = new ScoringState(pSwerveDrive, pIntake, pControls);
        mStates.put(scoringState.GetState(), scoringState);

        LowGoalScoringState lowGoalScoringState = new LowGoalScoringState(pSwerveDrive, pIntake, pControls);
        mStates.put(lowGoalScoringState.GetState(), lowGoalScoringState);

        mCurrentState = intakingState.GetState();
    }


     public void TransitionBackFromManualMode()
     {
        if(mCurrentState != null)
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
            
            mCurrentState = States.GameStart;
            AState newCurrentStateObject = mStates.get(mCurrentState);
            newCurrentStateObject.EnterState(null);
        }
    }

    public void Run()
    {
        AState currentStateObject = mStates.get(mCurrentState);
        SmartDashboard.putString("CurrentState", currentStateObject.GetName());
        NextStateInfo nextStateInfo = currentStateObject.Run();
        if(nextStateInfo.GetNextState() != mCurrentState)
        {            
            currentStateObject.ExitState();
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
    
}
