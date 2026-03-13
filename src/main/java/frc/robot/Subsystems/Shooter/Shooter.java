package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIO;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIO.ElevationIOInputs;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;

public class Shooter extends SubsystemBase{
    
    private FlywheelIO flywheelIO;
    private ElevationIO elevationIO;

    private ElevationIOInputsAutoLogged elevationInputs;
    private FlywheelIOInputsAutoLogged flywheelInputs;

    private double goalVelocity;
    private double goalAngle;

    private boolean atGoal;

    public enum ShooterSystemState {
        IDLE,
        ACTIVE_SHOOT_HUB,
        ACTIVE_SHOOT_PASSING,
        TRACKING_TARGET_HUB,
        TRACKING_TARGET_PASSING,
        ZERO
    }

    public enum ShooterWantedState {
        IDLE,
        ACTIVE_SHOOT_HUB,
        ACTIVE_SHOOT_PASSING,
        TRACKING_TARGET_HUB,
        TRACKING_TARGET_PASSING,
        ZERO
    }
    
    private ShooterSystemState systemState = ShooterSystemState.IDLE;
    private ShooterWantedState wantedState = ShooterWantedState.IDLE;

    public Shooter (ElevationIO elevationIO, FlywheelIO flywheelIO){
        this.elevationIO = elevationIO;
        this.flywheelIO = flywheelIO;

        goalVelocity = 0.0;
        goalAngle = 0.0;

        atGoal = false;
    }


    public ShooterSystemState handleStateTransitions(){
        switch (wantedState){
            case IDLE:
                return ShooterSystemState.IDLE;
                break;
            case ACTIVE_SHOOT_HUB:
                return ShooterSystemState.ACTIVE_SHOOT_HUB;
                break;
            case ACTIVE_SHOOT_PASSING:
                return ShooterSystemState.ACTIVE_SHOOT_PASSING;
                break;
            case TRACKING_TARGET_HUB:
                return ShooterSystemState.TRACKING_TARGET_HUB;
                break;
            case TRACKING_TARGET_PASSING:
                return ShooterSystemState.TRACKING_TARGET_PASSING;
                break;
            case ZERO:
                return ShooterSystemState.ZERO;
                break;
            default:
                return ShooterSystemState.IDLE;
                break;
        }
    }

    public void applyStates (){
        
    }

    public void setElevationAngle (Rotation2d angle){
        elevationIO.setElevationAngle(angle);
    }

    public void setFlywheelSpeed (AngularVelocity velo){
        flywheelIO.setVelo(velo);
    }



}
