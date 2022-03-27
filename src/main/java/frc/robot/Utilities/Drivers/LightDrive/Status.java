package frc.robot.Utilities.Drivers.LightDrive;

public class Status {
    private byte m_raw;
    
    private mode m_mode;
    
    public enum mode {
      NONE, IDLE, PWM, CAN, SERIAL;
    }
    
    public Status() {
      this.m_raw = 0;
      this.m_mode = mode.NONE;
    }
    
    public byte GetTripped() {
      return (byte)((this.m_raw & 0xF0) >> 4);
    }
    
    public Boolean IsEnabled() {
      return ((this.m_raw & 0x1) > 0) ? Boolean.valueOf(true) : Boolean.valueOf(false);
    }
    
    public mode GetMode() {
      return this.m_mode;
    }
    
    public Byte GetRaw() {
      return Byte.valueOf(this.m_raw);
    }
    
    public void SetRaw(byte raw) {
      this.m_raw = raw;
    }
  }
  