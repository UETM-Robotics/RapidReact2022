package frc.robot.Utilities.Drivers.LightDrive;

final class RxPacket {
    Status status = new Status();
    
    byte I1 = 0;
    
    byte I2 = 0;
    
    byte I3 = 0;
    
    byte I4 = 0;
    
    byte VIN = 0;
    
    byte PWMVals = 0;
    
    byte FW = 0;
    
    byte[] GetBytes() {
      byte[] tempdata = new byte[8];
      tempdata[0] = this.I1;
      tempdata[1] = this.I2;
      tempdata[2] = this.I3;
      tempdata[3] = this.I4;
      tempdata[4] = this.VIN;
      tempdata[5] = this.status.GetRaw().byteValue();
      tempdata[6] = this.PWMVals;
      tempdata[7] = this.FW;
      return tempdata;
    }
    
    void SetBytes(byte[] data) {
      this.I1 = data[0];
      this.I2 = data[1];
      this.I3 = data[2];
      this.I4 = data[3];
      this.VIN = data[4];
      this.status.SetRaw(data[5]);
      this.PWMVals = data[6];
      this.FW = data[7];
    }
  }
  