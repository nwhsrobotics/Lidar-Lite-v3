/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LidarLiteV3Subsystem extends Subsystem {

  private static final byte LIDAR_LITE_ADDR = 0x62;

  private static final byte REG_ACQ_COMMAND = 0x00;
  private static final byte ACQ_COMMAND_RESET = 0x00;
  private static final byte ACQ_COMMAND_MEAS_WITH_CORR = 0x04;

  private static final byte REG_ACQ_CONFIG = 0x04;
  // enable ref process, enable ref filter, disable quick term, use REF_COUNT_VAL, status output
  private static final byte ACQ_CONFIG_SETTING = 0x0D; 

  private static final byte REG_REF_COUNT_VAL = 0x12;
  private static final byte REF_COUNT_VAL_SETTING = 0x05;  // (default after reset is 5.)

  private static final byte WR_MEAS_WITH_CORR[] = {REG_ACQ_COMMAND, ACQ_COMMAND_MEAS_WITH_CORR};
  private static final byte WR_RESET[] = {REG_ACQ_COMMAND, ACQ_COMMAND_RESET};
 
  private static final byte WR_SET_ACQ_CONFIG[] = {REG_ACQ_CONFIG, ACQ_CONFIG_SETTING};
  private static final byte WR_SET_REF_COUNT[] = {REG_REF_COUNT_VAL, REF_COUNT_VAL_SETTING};

  private static final byte REG_STATUS = 0x01;
  private static final byte RD_STATUS[] = {REG_STATUS};

  private static final byte REG_FULL_DELAY = (byte)0x8F;
  private static final byte RD_FULL_DELAY_REG[] = {REG_FULL_DELAY};

  // A buffer for reading up to two bytes.
  private final byte m_buffer[] = new byte[2];

  boolean m_enabled = false;
  /* boolean m_measInProgress = false; */
  int m_distCm;
  int m_cycles = 0;

  I2C m_i2c = new I2C(I2C.Port.kOnboard, LIDAR_LITE_ADDR);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      /* if (m_measInProgress) { */
        if (!isBusy()) {
          // Read results of the measurement
          readMeas();
          /* m_measInProgress = false; */

          // Immediately start another measurement.
          startMeas();
          /* m_measInProgress = true; */
        }
      /* } */
      /* obsolete
      else {
        if (!isBusy()) {
          startMeas();
          m_measInProgress = true;
        }
      }
      */
    }
    else {
      m_distCm = 0;
    }

    SmartDashboard.putNumber("lidar_cm", m_distCm);
    SmartDashboard.putBoolean("lidar_en", m_enabled);
    SmartDashboard.putNumber("lidar_cycles", m_cycles);
  }

  public void init() {
    enable(false);
  }

  public void enable(boolean enable) {
    if (enable != m_enabled) {
      m_enabled = enable;

      if (m_enabled) {
        configLidar();
      }
      else {
        resetLidar();
      }
    }
  }

  public boolean isEnabled() {
    return m_enabled;
  }

  public double getDistanceCm() {
    // return last read distance
    return m_distCm;
  }

  private void configLidar() {
    // Set up ACQ_CONFIG register to use REF_COUNT instead of default value.
    m_i2c.writeBulk(WR_SET_REF_COUNT);
    m_i2c.writeBulk(WR_SET_ACQ_CONFIG);
  }


  private void resetLidar() {
    // Reset the Lidar, all registers return to default
    m_i2c.writeBulk(WR_RESET);
  }

  private void startMeas() {
    // Start one measurement
    m_i2c.writeBulk(WR_MEAS_WITH_CORR);
  }

  private void readMeas() {
    // read two bytes of FULL_DELAY
    m_i2c.writeBulk(RD_FULL_DELAY_REG);
    m_i2c.readOnly(m_buffer, 2);

    // convert two bytes to integer, units are cm.
    m_distCm = ((m_buffer[0] & 0xFF) << 8) + (m_buffer[1] & 0xFF);
    
    // Increment number of measurements so SmartDashboard shows progress.
    m_cycles += 1;
  }

  private boolean isBusy() {
    // Read STATUS register
    m_i2c.writeBulk(RD_STATUS);
    m_i2c.readOnly(m_buffer, 1);

    // If LSB is 1, we're busy.
    return ((m_buffer[0] & 0x01) == 1);
  }
}
