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
* \author  Tobias Bär <baer@fzi.de>
* \author  Dennis Nienhüser <nienhues@fzi.de>
* \date    15.03.2013
*
*/
//----------------------------------------------------------------------


#include "G25RacingWheel.h"
#include "Logging.h"
#include <icl_core_config/Config.h>

#include <fstream>
#include <iostream>

#include <fcntl.h>

using namespace std;
using namespace icl_core;

namespace icl_hardware {
namespace g25 {

G25RacingWheel::G25RacingWheel() : m_device(-1)
{
  string const device = config::getDefault<string> ("/Device", "/dev/input/by-id/usb-046d_G25_Racing_Wheel-event-joystick");
  if ((m_device = open(device.c_str() , O_RDWR | O_NONBLOCK)) < 0)
  {
    G25_ERROR("Unable to open device " << device << ". Are you sure it's plugged in? Please check permissions as well.");
    return;
  }

  int version = 0x000800;
  if (ioctl(m_device, EVIOCGVERSION, &version))
  {
    G25_ERROR("Unable to determine the version of the racing wheel!");
    return;
  }

  unsigned short id[4];
  ioctl(m_device, EVIOCGID, id);

  char name[256] = "Unknown";
  ioctl(m_device, EVIOCGNAME(sizeof(name)), name);
  G25_DEBUG("\tBus Id:\t\t" << id[ID_BUS]);
  G25_DEBUG("\tVendor Id:\t" << id[ID_VENDOR]);
  G25_DEBUG("\tProduct Id:\t" << id[ID_PRODUCT]);
  G25_DEBUG("\tVersion Id:\t" << id[ID_VERSION]);
  G25_DEBUG("\tName:\t\t" << name);

  int abs[5];
  ioctl(m_device, EVIOCGABS(0), abs);
  m_steering_max = abs[1];
  m_steering_range_max = abs[2];
  G25_INFO("Connected to G25 Racing Wheel");
}

void G25RacingWheel::readRacingWheel()
{
  if(-1 == m_device)
  {
    return;
  }

  struct input_event ev[64];
  memset(ev, 0, 64 * sizeof(input_event));

  int const num_read = read(m_device, ev , sizeof(struct input_event) * 64);
  if(num_read > 0)
  {
    int const events_read = num_read / sizeof(struct input_event);
    for(int i=0; i<events_read; ++i)
    {
      parseEvent(ev[i]);
      G25_TRACE("Event: " << ev[i].time.tv_sec << " " << ev[i].type << " " << ev[i].code << " " << ev[i].value);
    }
  }
}

void G25RacingWheel::buttonPress(G25RacingWheel::ButtonType button, G25RacingWheel::ButtonPress press)
{
  G25_TRACE("Button type " << button << (press == Down ? "down" : "up"));
}

void G25RacingWheel::steer(double angle)
{
  G25_TRACE("Steering to " << angle);
}

void G25RacingWheel::accelerate(double value)
{
  G25_TRACE("Accelerating with " << int(value*100) << "%");
}

void G25RacingWheel::brake(double value)
{
  G25_TRACE("Braking with " << int(value*100) << "%");
}

void G25RacingWheel::getGear(G25RacingWheel::Gear gear, G25RacingWheel::ButtonPress press)
{
  if (isIdle(press))
  {
    return;
  }
  G25_TRACE("Gear " << gear << (press == Down ? " deactivated" : " active"));
}

void G25RacingWheel::parseEvent(const struct input_event& event)
{
  //std::cout << "code=" << event.code << " signal=" << event.type << std::endl;
  if(EV_SYN == event.type) //0
  {
    if(0 == event.code)
    {
      // Reset signal. Nothing to do.
    }
  }
  else if (1 == event.type) //1
  {
    //cout << "code: " << event.code << endl;
    //cout << "the type: " << event.type << endl;
    switch (event.code)
    {   
    case 295: buttonPress(WheelButonLeft, event.value == 1 ? Up : Down); return;
    case 294: buttonPress(WheelButtonRight, event.value == 1 ? Up : Down); return;
    case 293: buttonPress(ShiftPaddelLeft, event.value == 1 ? Up : Down); return;
    case 292: buttonPress(ShiftPaddelRight, event.value == 1 ? Up : Down); return;
    case 299: buttonPress(RedButton1, event.value == 1 ? Up : Down); return;
    case 296: buttonPress(RedButton2, event.value == 1 ? Up : Down); return;
    case 297: buttonPress(RedButton3, event.value == 1 ? Up : Down); return;
    case 298: buttonPress(RedButton4, event.value == 1 ? Up : Down); return;
    case 291: buttonPress(BlackButtonTop, event.value == 1 ? Up : Down); return;
    case 288: buttonPress(BlackButtonDown, event.value == 1 ? Up : Down); return;
    case 290: buttonPress(BlackButtonRight, event.value == 1 ? Up : Down); return;
    case 289: buttonPress(BlackButtonLeft, event.value == 1 ? Up : Down); return;
    case 706: getGear(REVERSE, event.value == 1 ? Up : Down); return;
    case 300: getGear(ONE, event.value == 1 ? Up : Down); return;
    case 301: getGear(TWO, event.value == 1 ? Up : Down); return;
    case 302: getGear(THREE, event.value == 1 ? Up : Down); return;
    case 303: getGear(FOUR, event.value == 1 ? Up : Down); return;
    case 704: getGear(FIVE, event.value == 1 ? Up : Down); return;
    case 705: getGear(SIX, event.value == 1 ? Up : Down); return;
    }
  }
  else if(EV_ABS == event.type) //3
  {
    if(0 == event.code)
    {
      steer(mapValueLinear(m_steering_max, m_steering_range_max, -450.0, 450.0, event.value));
    }

    if(2 == event.code)
    {
      accelerate(1.0-mapValueLinear(0, 255, 0.0, 1.0, event.value));
    }

    if(5 == event.code)
    {
      brake(1.0-mapValueLinear(0, 255, 0.0, 1.0, event.value));
    }
  }
}

float G25RacingWheel::mapValueLinear(int hardware_min, int hardware_max, float mapping_min, float mapping_max, int value) const
{
  return (((float(value-hardware_min)) / (hardware_max-hardware_min)) * (mapping_max-mapping_min)) + mapping_min;
}

int G25RacingWheel::mapGear(Gear gear)
{
    switch(gear)
    {
    case REVERSE: return int(-1);
    case IDLE: return int(0);
    case ONE: return int(1);
    case TWO: return int(2);
    case THREE: return int(3);
    case FOUR: return int(4);
    case FIVE: return int(5);
    case SIX: return int(6);
    }
}

bool G25RacingWheel::isIdle(G25RacingWheel::ButtonPress press)
{
    if (press == Down)
    {
        G25_TRACE("Idle" << IDLE << " active");
        return true;
    }
    return false;
}
}
}
