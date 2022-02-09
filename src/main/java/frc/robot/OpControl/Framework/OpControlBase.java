package frc.robot.OpControl.Framework;

public abstract class OpControlBase {

    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;

    protected abstract void routine() throws OpControlEndedException;

    public void run() {
        m_active = true;

        try {
            routine();
        } catch(OpControlEndedException e) {
            System.out.println("OpControl mode done, ended early");
            return;
        }

        done();
        System.out.println("OpControl Mode Done");
    }

    public void done() {}

    public void stop() {
        m_active = false;
    }
    
    public boolean isActive() {
        return m_active;
    }
}
