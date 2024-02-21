// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorConstants;

public class Notification extends SubsystemBase {
  public enum NoteState {
    NOTE_IN_POSSESSION,
    NOTE_NOT_IN_POSSESSION
  }

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private NoteState noteState = NoteState.NOTE_NOT_IN_POSSESSION;

  /** Creates a new Intake. */
  public Notification() {
    m_led = new AddressableLED(0);                // 0 = number of port on three letter thing i forgot what it called
    m_ledBuffer = new AddressableLEDBuffer(10); // 1 = number of leds in length of it
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    updateState(NoteState.NOTE_NOT_IN_POSSESSION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateState(NoteState noteState) {
    int hue = 0;
    if (noteState == NoteState.NOTE_IN_POSSESSION) {
      hue = ColorConstants.BlueHue;
    } else if (noteState == NoteState.NOTE_NOT_IN_POSSESSION) {
      hue = ColorConstants.RedHue;
    }
  
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(i, hue, 255, 255);   // could also do .setRGB if we want that color system
    }
  
    m_led.setData(m_ledBuffer);
    this.noteState = noteState;
  }

  public NoteState getNoteState() {
    return noteState;
  }
}
