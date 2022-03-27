package frc.robot.Utilities.Drivers.LightDrive;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.util.UncleanStatusException;
import java.awt.Color;
import java.util.Vector;

public final class LightDrive2812 {
  static final int LED_COUNT = 512;
  
  static final int LD_ADDR = 33882368;
  
  static final int BUFFER_LIMIT = 16;
  
  static final int LIB_VERSION = 3;
  
  long last_time;
  
  int m_image_size;
  
  byte[] data;
  
  Vector<TX_Packet> tx_buffer;
  
  boolean debugEnabled;
  
  public LightDrive2812() {
    this.debugEnabled = false;
    this.tx_buffer = new Vector<>();
    this.last_time = System.currentTimeMillis();
    SetRange(Color.BLACK, 0, 511);
    Update();
  }
  
  public int GetVersion() {
    return 3;
  }
  
  public void DebugEnable() {
    this.debugEnabled = true;
  }
  
  public void DebugDisable() {
    this.debugEnabled = false;
  }
  
  public void ClearLEDs() {
    SetRange(Color.BLACK, 0, 511);
  }
  
  public void Update() {
    try {
      if (this.tx_buffer.size() > 0 && 
        System.currentTimeMillis() - this.last_time > 15L) {
        this.last_time = System.currentTimeMillis();
        CANJNI.FRCNetCommCANSessionMuxSendMessage(33882368, ((TX_Packet)this.tx_buffer.remove(0)).GetRaw(), 0);
        if (this.debugEnabled)
          System.out.println("Sent CAN message. Buffer size " + Integer.toString(this.tx_buffer.size())); 
      } 
    } catch (UncleanStatusException e) {
      System.err.println("LightDrive CAN: Unclean Status Exception " + e.getMessage());
    } 
  }
  
  public void SetRange(Color color, int start, int count) {
    TX_Packet tx = new TX_Packet();
    if (start >= 0 && count > 0 && start + count < 512) {
      tx.red = (byte)color.getRed();
      tx.green = (byte)color.getGreen();
      tx.blue = (byte)color.getBlue();
      tx.offset = (short)start;
      tx.length = (short)count;
      if (this.tx_buffer.size() > 16) {
        if (this.debugEnabled)
          System.out.println("LightDrive buffer overflow. Overwritting..."); 
        this.tx_buffer.remove(0);
      } 
      this.tx_buffer.add(tx);
      if (this.debugEnabled)
        System.out.println("Added setRange() at " + Integer.toString(this.tx_buffer.size() - 1)); 
    } 
  }
  
  public void SetRangeandClear(Color color, int start, int count, int outOf) {
    if (count > 0 && outOf < 512 && count <= outOf && start < count) {
      SetRange(color, start, count);
      SetRange(Color.BLACK, count, outOf - count);
    } 
  }
  
  private class TX_Packet {
    private byte[] raw = new byte[8];
    
    public byte red;
    
    public byte green;
    
    public byte blue;
    
    public short offset;
    
    public short length;
    
    public byte[] GetRaw() {
      this.raw[0] = this.red;
      this.raw[1] = this.green;
      this.raw[2] = this.blue;
      this.raw[3] = (byte)(this.offset >> 8 & 0xFF);
      this.raw[4] = (byte)(this.offset & 0xFF);
      this.raw[5] = (byte)(this.length >> 8 & 0xFF);
      this.raw[6] = (byte)(this.length & 0xFF);
      this.raw[7] = 0;
      return this.raw;
    }
  }
}
