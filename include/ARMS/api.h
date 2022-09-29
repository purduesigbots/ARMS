#pragma once

#include "ARMS/lib.h"

// The config file needs to be last because the arms::init()
// function inside of it depends on init() function of the subsystems
#include "ARMS/config.h"
