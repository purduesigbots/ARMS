/**
 * Implementation file for ARMS Lua support.
 * 
 * As of right now, this is a simple solution to add Lua support to ARMS
 */

#include "ARMS/lua.h"

#include "ARMS/chassis.h"


/* Actual implementation for the ARMS lua interface */
namespace arms::lua {

bool init() {
    lua_State* state = luaL_newstate();
}

} /* namespace arms::lua */