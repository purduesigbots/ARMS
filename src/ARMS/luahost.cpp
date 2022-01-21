/**
 * Host functions for Lua
 */

#include "ARMS/Lua.h"
#include "ARMS/Chassis.h"

namespace chassis = arms::chassis; /* Preferable to "using namespace arms" */

/**
 * This namespace contains wrappers for all of the ARMS functions that will be
 * visible for the LUA scripting
 */
namespace host {
    int resetAngle(lua_State* s) {
        chassis::resetAngle(lua_tonumber(s, 1));
        return 0;
    }

    int reset(lua_State* s) {
        chassis::reset();
        return 0;
    }

    int position(lua_State* s) {
        return 0;
    }

    int angle(lua_State* s) {
        return 0;
    }

    int difference(lua_State* s) {
        return 0;
    }

    int limitSpeed(lua_State* s) {
        return 0;
    }

    int slew(lua_State* s) {
        return 0;
    }
    int settled(lua_State* s) {
        return 0;
    }

    int waitUntilSettled(lua_State* s) {
        return 0;
    }

    int moveAsync(lua_State* s) {
        return 0;
    }

    int turnAsync(lua_State* s) {
        return 0;
    }

    int turnAbsoluteAsync(lua_State* s) {
        return 0;
    }

    int holoAsync(lua_State* s) {
        return 0;
    }

    int move(lua_State* s) {
        return 0;
    }

    int turn(lua_State* s) {
        return 0;
    }

    int turnAbsolute(lua_State* s) {
        return 0;
    }

    int holo(lua_State* s) {
        return 0;
    }

    int fast(lua_State* s) {
        return 0;
    }

    int voltage(lua_State* s) {
        return 0;
    }

    int velocity(lua_State* s) {
        chassis::velocity(
            lua_tointeger(s, 1),
            lua_tointeger(s, 2),
            lua_tointeger(s, 3),
        );

        return 0;
    }

    int tank(lua_State* s) {
        chassis::arcade(lua_tonumber(s, 1), lua_tonumber(s, 2));
        return 0;
    }

    int arcade(lua_State* s) {
        chassis::arcade(lua_tonumber(s, 1), lua_tonumber(s, 2));
        return 0;
    }
    
    int holonomic(lua_State* s) {
        chassis::holonomic(lua_tonumber(s, 1), lua_tonumber(s, 2));
        return 0; /* This returns nothing to lua */
    }
} /* namespace internal */
