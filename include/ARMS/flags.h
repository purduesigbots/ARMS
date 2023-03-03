#ifndef _ARMS_FLAGS_H_
#define _ARMS_FLAGS_H_

namespace arms {

struct MoveFlags {

	bool async;
	bool true_relative;
	bool thru;
	bool reverse;
	bool relative;

	MoveFlags operator|(MoveFlags& o) {
		MoveFlags ret;
		ret.async = async || o.async;
		ret.true_relative = true_relative || o.true_relative;
		ret.thru = thru || o.thru;
		ret.reverse = reverse || o.reverse;
		ret.relative = relative || o.relative;
		return ret;
	}

	MoveFlags operator&(MoveFlags& o) {
		MoveFlags ret;
		ret.async = async && o.async;
		ret.true_relative = true_relative && o.true_relative;
		ret.thru = thru && o.thru;
		ret.reverse = reverse && o.reverse;
		ret.relative = relative && o.relative;
		return ret;
	}

	MoveFlags operator~() {
		MoveFlags ret;
		ret.async = !async;
		ret.true_relative = !true_relative;
		ret.thru = !thru;
		ret.reverse = !reverse;
		ret.relative = !relative;
		return ret;
	}

	MoveFlags operator|(const MoveFlags& o) {
		MoveFlags ret;
		ret.async = async || o.async;
		ret.true_relative = true_relative || o.true_relative;
		ret.thru = thru || o.thru;
		ret.reverse = reverse || o.reverse;
		ret.relative = relative || o.relative;
		return ret;
	}


	operator bool() {
		return async || true_relative || thru || reverse || relative;
	}
};

inline MoveFlags operator|(const MoveFlags& f, MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async || o.async;
	ret.true_relative = f.true_relative || o.true_relative;
	ret.thru = f.thru || o.thru;
	ret.reverse = f.reverse || o.reverse;
	ret.relative = f.relative || o.relative;
	return ret;
}

inline MoveFlags operator&(const MoveFlags& f, MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async && o.async;
	ret.true_relative = f.true_relative && o.true_relative;
	ret.thru = f.thru && o.thru;
	ret.reverse = f.reverse && o.reverse;
	ret.relative = f.relative && o.relative;
	return ret;
}

inline MoveFlags operator|(const MoveFlags& f, const MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async || o.async;
	ret.true_relative = f.true_relative || o.true_relative;
	ret.thru = f.thru || o.thru;
	ret.reverse = f.reverse || o.reverse;
	ret.relative = f.relative || o.relative;
	return ret;
}

inline MoveFlags operator&(const MoveFlags& f, const MoveFlags& o) {
	MoveFlags ret;
	ret.async = f.async && o.async;
	ret.true_relative = f.true_relative && o.true_relative;
	ret.thru = f.thru && o.thru;
	ret.reverse = f.reverse && o.reverse;
	ret.relative = f.relative && o.relative;
	return ret;
}

const MoveFlags NONE = {false, false, false, false, false};
const MoveFlags ASYNC = {true, false, false, false, false};
const MoveFlags TRUE_RELATIVE = {false, true, false, false, false};
const MoveFlags THRU = {false, false, true, false, false};
const MoveFlags REVERSE = {false, false, false, true, false};
const MoveFlags RELATIVE = {false, false, false, false, true};

const MoveFlags NOT_ASYNC = {false, true, true, true, true};
const MoveFlags NOT_TRUE_RELATIVE = {true, false, true, true, true};
const MoveFlags NOT_THRU = {true, true, false, true, true};
const MoveFlags NOT_REVERSE = {true, true, true, false, true};
const MoveFlags NOT_RELATIVE = {true, true, true, true, false};


} // namespace arms

#endif