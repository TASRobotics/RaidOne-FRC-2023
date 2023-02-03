package frc.robot.auto.actions;

/** Shamelessly stolen from Raid Zero 2020 code */
public class LambdaAction implements Action {

    public interface VoidInterface {
        void f();
    }

    VoidInterface mF;

    public LambdaAction(VoidInterface f) {
        this.mF = f;
    }

    @Override
    public void initialize() {
        mF.f();
    }

    @Override
    public void update() {}

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public void done() {}
}