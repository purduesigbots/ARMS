#ifndef _ARMS_FLAGS_H_
#define _ARMS_FLAGS_H_

namespace arms {

struct MoveFlags {

	bool async;
	bool thru;
	bool reverse;

	MoveFlags operator|(MoveFlags& o) {
		MoveFlags ret;
		ret.async = async || o.async;
		ret.thru = thru || o.thru;
		ret.reverse = reverse || o.reverse;
		return ret;
	}

	MoveFlags operator&(MoveFlags& o) {
		MoveFlags ret;
		ret.async = async && o.async;
		ret.thru = thru && o.thru;
		ret.reverse = reverse && o.reverse;
		return ret;
	}

	MoveFlags operator|(const MoveFlags& o) {
		MoveFlags ret;
		ret.async = async || o.async;
		ret.thru = thru || o.thru;
		ret.reverse = reverse || o.reverse;
		return ret;
	}

	MoveFlags operator&(const MoveFlags& o) {
		MoveFlags ret;
		ret.async = async && o.async;
		ret.thru = thru && o.thru;
		ret.reverse = reverse && o.reverse;
		return ret;
	}

	operator bool() {
		return async || thru || reverse;
	}
};

inline MoveFlags operator|(const MoveFlags& f, MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async || o.async;
	ret.thru = f.thru || o.thru;
	ret.reverse = f.reverse || o.reverse;
	return ret;
}

inline MoveFlags operator&(const MoveFlags& f, MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async && o.async;
	ret.thru = f.thru && o.thru;
	ret.reverse = f.reverse && o.reverse;
	return ret;
}

inline MoveFlags operator|(const MoveFlags& f, const MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async || o.async;
	ret.thru = f.thru || o.thru;
	ret.reverse = f.reverse || o.reverse;
	return ret;
}

inline MoveFlags operator&(const MoveFlags& f, const MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async && o.async;
	ret.thru = f.thru && o.thru;
	ret.reverse = f.reverse && o.reverse;
	return ret;
}

const MoveFlags NONE = {false, false, false};
const MoveFlags ASYNC = {true, false, false};
const MoveFlags THRU = {false, true, false};
const MoveFlags BACKWARDS = {false, false, true};

} // namespace arms

#endif