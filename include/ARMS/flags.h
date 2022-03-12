#ifndef _ARMS_FLAGS_H_
#define _ARMS_FLAGS_H_

namespace arms {

struct MoveFlags {

	bool async;
	bool relative;
	bool thru;
	bool reverse;

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
};

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

const MoveFlags NONE = {false, false, false, false};
const MoveFlags ASYNC = {true, false, false, false};
const MoveFlags RELATIVE = {false, true, false, false};
const MoveFlags THRU = {false, false, true, false};
const MoveFlags REVERSE = {false, false, false, true};

} // namespace arms

#endif