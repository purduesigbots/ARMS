#ifndef _ARMS_FLAGS_H_
#define _ARMS_FLAGS_H_

namespace arms {

/*!
	* @struct MoveFlags
	*
	* @details This struct contains the flags that are used to control the movement of the robot.\n
	* There are 4 different flags for this struct, each of which is a boolean value.\n
	*
	* The \a ASYNC flag will cause the movement to be asynchronous, meaning that the robot will not wait for the movement to finish before continuing the code.\n
	* The \a RELATIVE flag will cause the movement to be relative to the robot's current position, instead of global odom coordinates.\n
	* The \a THRU flag will disable PID control for the movement, causing the robot to move at maximum speed for that movement.\n
	* The \a REVERSE flag will cause the robot to move towards the movement using it's "back" instead of it's "front". If you tell the robot to move to a point behind it without this flag, it will turn around and move to the point instead of moving backwards.\n 
	*
	* <b>Example 1:</b>
	* @code
	* // create a MoveFlags object that is asynchronous and relative
	* MoveFlags option_1 = {true, true, false, false}; // create with vector
	* MoveFlags option_2 = arms::ASYNC | arms::RELATIVE; // create with flags
	* @endcode
	*
	* <b>Example 2:</b>
	* @code
	* // perform a chassis movement that is reverse, relative, and asynchronous
	* chassis.move({0, 0}, 100, {true, true, false, true}); // move with vector
	* chassis.move({0, 0}, 100, arms::REVERSE | arms::RELATIVE | arms::ASYNC); // move with flags
	* @endcode
	*
	* @var boolean MoveFlags::async 
	* Determines if the movement is asynchronous or not.
	* @var boolean MoveFlags::relative 
	* Determines if the movement is relative or not.
	* @var boolean MoveFlags::thru 
	* Determines if the movement should use PID or not.
	* @var boolean MoveFlags::reverse 
	* Determines if the movement is reverse or not.
*/
struct MoveFlags {

	bool async;
	bool relative;
	bool thru;
	bool reverse;
/// @cond DO_NOT_DOCUMENT
	MoveFlags operator|(MoveFlags& o) {
		MoveFlags ret;
		ret.async = async || o.async;
		ret.relative = relative || o.relative;
		ret.thru = thru || o.thru;
		ret.reverse = reverse || o.reverse;
		return ret;
	}

	MoveFlags operator&(MoveFlags& o) {
		MoveFlags ret;
		ret.async = async && o.async;
		ret.relative = relative && o.relative;
		ret.thru = thru && o.thru;
		ret.reverse = reverse && o.reverse;
		return ret;
	}

	MoveFlags operator|(const MoveFlags& o) {
		MoveFlags ret;
		ret.async = async || o.async;
		ret.relative = relative || o.relative;
		ret.thru = thru || o.thru;
		ret.reverse = reverse || o.reverse;
		return ret;
	}

	MoveFlags operator&(const MoveFlags& o) {
		MoveFlags ret;
		ret.async = async && o.async;
		ret.relative = relative && o.relative;
		ret.thru = thru && o.thru;
		ret.reverse = reverse && o.reverse;
		return ret;
	}

	operator bool() {
		return async || relative || thru || reverse;
	}
	/// @endcond
};
/// @cond DO_NOT_DOCUMENT
inline MoveFlags operator|(const MoveFlags& f, MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async || o.async;
	ret.relative = f.relative || o.relative;
	ret.thru = f.thru || o.thru;
	ret.reverse = f.reverse || o.reverse;
	return ret;
}

inline MoveFlags operator&(const MoveFlags& f, MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async && o.async;
	ret.relative = f.relative && o.relative;
	ret.thru = f.thru && o.thru;
	ret.reverse = f.reverse && o.reverse;
	return ret;
}

inline MoveFlags operator|(const MoveFlags& f, const MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async || o.async;
	ret.relative = f.relative || o.relative;
	ret.thru = f.thru || o.thru;
	ret.reverse = f.reverse || o.reverse;
	return ret;
}

inline MoveFlags operator&(const MoveFlags& f, const MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async && o.async;
	ret.relative = f.relative && o.relative;
	ret.thru = f.thru && o.thru;
	ret.reverse = f.reverse && o.reverse;
	return ret;
}
/// @endcond

/*!
 * @brief This is a constant that can be used to specify that no flags are used.
 */
const MoveFlags NONE = {false, false, false, false};
/*!
 * @brief This is a constant that can be used to specify that the \a async flag is used.
 */
const MoveFlags ASYNC = {true, false, false, false};
/*!
 * @brief This is a constant that can be used to specify that the \a relative flag is used.
 */
const MoveFlags RELATIVE = {false, true, false, false};
/*!
 * @brief This is a constant that can be used to specify that the \a thru flag is used.
 */
const MoveFlags THRU = {false, false, true, false};
/*!
 * @brief This is a constant that can be used to specify that the \a reverse flag is used.
 */
const MoveFlags REVERSE = {false, false, false, true};

} // namespace arms

#endif