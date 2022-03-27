package frc.robot.Utilities.Drivers.LightDrive;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.hal.util.UncleanStatusException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public final class LightDriveCAN {
  public LightDriveCAN() {
    this.m_matrix = ByteBuffer.allocate(16);
    this.m_init = false;
    this.m_rx = new RxPacket();
    timestamp = ByteBuffer.allocateDirect(4);
    timestamp.order(ByteOrder.LITTLE_ENDIAN);
    rxid = ByteBuffer.allocateDirect(4);
    rxid.order(ByteOrder.LITTLE_ENDIAN);
  }
  
  public LightDriveCAN(int addr) {}
  
  public void Update() {
    byte[] txdata = new byte[8];
    rxid.putInt(LD_ADDR + 4);
    rxid.rewind();
    try {
      this.m_matrix.get(txdata, 0, 8);
      CANJNI.FRCNetCommCANSessionMuxSendMessage(LD_ADDR, txdata, 100);
      this.m_matrix.get(txdata, 0, 8);
      CANJNI.FRCNetCommCANSessionMuxSendMessage(LD_ADDR + 1, txdata, 100);
    } catch (UncleanStatusException uncleanStatusException) {}
    this.m_matrix.rewind();
    try {
      this.rxdata = CANJNI.FRCNetCommCANSessionMuxReceiveMessage(rxid.asIntBuffer(), 536870911, timestamp);
      if (this.rxdata.length > 7)
        this.m_rx.SetBytes(this.rxdata); 
    } catch (CANMessageNotFoundException cANMessageNotFoundException) {}
  }
  
  public float GetCurrent(int ch) {
    float current = 0.0F;
    switch (ch) {
      case 1:
        current = this.m_rx.I1;
        return current / 10.0F;
      case 2:
        current = this.m_rx.I2;
        return current / 10.0F;
      case 3:
        current = this.m_rx.I3;
        return current / 10.0F;
      case 4:
        current = this.m_rx.I4;
        return current / 10.0F;
    } 
    current = -10.0F;
    return current / 10.0F;
  }
  
  public float GetTotalCurrent() {
    return (this.m_rx.I1 + this.m_rx.I2 + this.m_rx.I3 + this.m_rx.I4) / 10.0F;
  }
  
  public float GetVoltage() {
    return this.m_rx.VIN / 10.0F;
  }
  
  public int GetFWVersion() {
    return this.m_rx.FW;
  }
  
  public Status GetStatus() {
    return this.m_rx.status;
  }
  
  public int GetPWMs(int ch) {
    if (ch > 2 || ch < 1)
      return -1; 
    return (ch > 1) ? (this.m_rx.PWMVals >> 8) : (this.m_rx.PWMVals & 0xFF);
  }
  
  public void SetColor(int ch, Color color) {
    if (ch < 1 || ch > 4)
      return; 
    ch--;
    ch *= 3;
    this.m_matrix.array()[ch] = color.green;
    this.m_matrix.array()[ch + 1] = color.red;
    this.m_matrix.array()[ch + 2] = color.blue;
  }
  
  public void SetColor(int ch, Color color, double brightness) {
    if (ch < 1 || ch > 4)
      return; 
    color.red = (byte)(int)(color.red * brightness);
    color.green = (byte)(int)(color.green * brightness);
    color.blue = (byte)(int)(color.blue * brightness);
    ch--;
    ch *= 3;
    this.m_matrix.array()[ch] = color.green;
    this.m_matrix.array()[ch + 1] = color.red;
    this.m_matrix.array()[ch + 2] = color.blue;
  }
  
  public void SetLevel(int ch, byte level) {
    if (ch < 1 || ch > 12 || level < 0 || level > 255)
      return; 
    this.m_matrix.array()[ch] = level;
  }
  
  private static int LD_ADDR = 33882112;
  
  private ByteBuffer m_matrix;
  
  private RxPacket m_rx;
  
  private boolean m_init;
  
  private byte[] rxdata;
  
  private static ByteBuffer timestamp;
  
  private static ByteBuffer rxid;
}