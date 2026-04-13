package frc.robot.Subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Indexer extends SubsystemBase {

  private IndexerIO indexerIO;

  private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  public enum IndexerSystemState {
    IDLE,
    RUNNING
  }

  public enum IndexerWantedState {
    IDLE,
    RUNNING
  }

  @AutoLogOutput private IndexerSystemState systemState = IndexerSystemState.IDLE;
  @AutoLogOutput private IndexerWantedState wantedState = IndexerWantedState.IDLE;

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerInputs);

    systemState = handleStateTransitions();

    applyStates();
  }

  public IndexerSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return IndexerSystemState.IDLE;
      case RUNNING:
        return IndexerSystemState.RUNNING;
      default:
        return IndexerSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        indexerIO.setVoltage(0);
        break;
      case RUNNING:
        // find good numbers
        indexerIO.setMotor1Voltage(3);
        indexerIO.setMotor2Voltage(1.5);
        break;
      default:
        indexerIO.setVoltage(0);
        break;
    }
  }

  public void setWantedState(IndexerWantedState wantedState) {
    this.wantedState = wantedState;
  }
}
