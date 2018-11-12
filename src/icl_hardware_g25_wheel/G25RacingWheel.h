// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Dennis Nienhüser <nienhues@fzi.de>
* \date    15.03.2013
*
*/
//----------------------------------------------------------------------

#ifndef ICL_HARDWARE_G25_RACING_WHEEL_H
#define ICL_HARDWARE_G25_RACING_WHEEL_H

#include "ImportExport.h"

#include <linux/input.h>

namespace icl_hardware {
namespace g25 {

class ICL_HARDWARE_G25_WHEEL_IMPORT_EXPORT G25RacingWheel
{
public:
  enum ButtonPress
  {
    Up,
    Down
  };

  enum ButtonType
  {
    WheelButonLeft,
    WheelButtonRight,
    ShiftPaddelLeft,
    ShiftPaddelRight,
    GearUp,
    GearDown,
    RedButton1,
    RedButton2,
    RedButton3,
    RedButton4,
    BlackButtonTop,
    BlackButtonDown,
    BlackButtonRight,
    BlackButtonLeft
  };

  enum Gear
  {
      REVERSE = -1,
      IDLE    = 0,
      ONE     = 1,
      TWO     = 2,
      THREE   = 3,
      FOUR    = 4,
      FIVE    = 5,
      SIX     = 6
  };

  /** Constructor. Connects to the device */
  G25RacingWheel();

  /** Read buffered events from the device, calls gearUp, gearDown etc. as
   * needed and returns (does not block)
   */
  void readRacingWheel();

  /** Overwrite this to get button press (up and down) events */
  virtual void buttonPress(ButtonType button, ButtonPress press);

  /** Overwrite this to get steering change events. Absolute value in degree */
  virtual void steer(double angle);

  /** Overwrite this to get accelerate change events. Absolute value from 0..1 */
  virtual void accelerate(double value);

  /** Overwrite this to get brake change events. Absolute value from 0..1 */
  virtual void brake(double value);

  /** Overwrite this to get gear. Value ReverseGear, One..Six and if pressed/in use */
  virtual void getGear(Gear gear, ButtonPress press);

private:
  void parseEvent( const struct input_event& event);
  float mapValueLinear(int hardware_min, int hardware_max, float mapping_min, float mapping_max, int value) const;
  int mapGear(Gear gear);
  bool isIdle(G25RacingWheel::ButtonPress press);

  int m_device;
  int m_steering_max;
  int m_steering_range_max;
};

}
}

#endif
