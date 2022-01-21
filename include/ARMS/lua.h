/**
 * Experimental Lua scripting support for ARMS.
 */

#ifndef _ARMS_LUA_H_
#define _ARMS_LUA_H_

// Include the proper files for the Lua interpreter
// It is important lua/lua.hpp is used and not lua/lua.h because
// the .hpp file wraps the headers in an extern "C" declaration
#include "lua/lua.hpp"

namespace arms::lua {
    bool init();
}

#endif//_ARMS_LUA_H_