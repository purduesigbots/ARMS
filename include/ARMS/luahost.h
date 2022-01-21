#ifndef _ARMS_LUA_HOST_H_
#define _ARMS_LUA_HOST_H_

#include "lua/lua.hpp"

/**
 * Every function in this namespace is a wrapper to it's corresponding ARMS
 * function. These wrappers convert the Lua host-function calls into the actual
 * calls for ARMS functions. Every single host function takes pointer to a
 * lua_State structure, which is used to look at variables on the stack, and
 * then returns the number of variables being returned back to the Lua machine.
 * 
 */
namespace lua::host {
    //functions defined in ARMS/chassis.h
    int resetAngle(lua_State* s);
    int reset(lua_State* s);
    int position(lua_State* s);
    int angle(lua_State* s);
    int difference(lua_State* s);
    int limitSpeed(lua_State* s);
    int slew(lua_State* s);
    int settled(lua_State* s);
    int waitUntilSettled(lua_State* s);
    int moveAsync(lua_State* s);
    int turnAsync(lua_State* s);
    int turnAbsoluteAsync(lua_State* s);
    int holoAsync(lua_State* s);
    int move(lua_State* s);
    int turn(lua_State* s);
    int turnAbsolute(lua_State* s);
    int holo(lua_State* s);
    int fast(lua_State* s);
    int voltage(lua_State* s);
    int velocity(lua_State* s);
    int tank(lua_State* s);
    int arcade(lua_State* s);
    int holonomic(lua_State* s);
}

#endif//_ARMS_LUA_HOST_H_