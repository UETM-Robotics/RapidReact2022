package frc.robot.Utilities.Drivers.LightDrive;

public final class Color {
  public byte red;
  
  public byte green;
  
  public byte blue;
  
  public static final Color RED = new Color(255, 0, 0);
  
  public static final Color GREEN = new Color(0, 255, 0);
  
  public static final Color BLUE = new Color(0, 0, 255);
  
  public static final Color TEAL = new Color(0, 255, 255);
  
  public static final Color YELLOW = new Color(255, 255, 0);
  
  public static final Color PURPLE = new Color(255, 0, 255);
  
  public static final Color WHITE = new Color(255, 255, 255);
  
  public static final Color OFF = new Color(0, 0, 0);
  
  public Color() {
    this.red = 0;
    this.green = 0;
    this.blue = 0;
  }
  
  public Color(int r, int g, int b) {
    this.red = (byte)r;
    this.green = (byte)g;
    this.blue = (byte)b;
  }
}
