#pragma once

namespace copilot
{

namespace states
{
const int INVALID = -1;
const int OFF = 0;
const int OPTITRACK_START = 1;
const int VISION_START = 2;
const int HOVER = 3;
const int OPTITRACK_LAND = 4;
const int VISION_LAND = 5;
const int EMERGENCYLAND = 6;
const int WAIT_FOR_USER_CONTROLLER = 7;
const int FEEDTHROUGH = 8;
const int BREAKING = 9;
const int RC_MANUAL = 10;
}

}
