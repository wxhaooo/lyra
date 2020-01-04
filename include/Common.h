#pragma once
#include<memory>
#include<typeinfo>
#include<algorithm>
#include<cmath>
using uint32 = unsigned int;
using uint32_pt = std::shared_ptr<uint32>;

//float M_PI = 3.141592653;

constexpr float PI = 3.141592653f;

inline float Angle2Radian(float angle) { return PI / 180.f * angle; }
inline float Radian2Angle(float radian) { return 180.f / PI * radian; }

inline bool IsZero(float v)
{
	if (std::abs(v) < 10e-8) return true;
	return false;
}

inline float SafeAcos(float radian)
{
	if (radian < -1.f) radian = -1.f;
	else if (radian > 1.f) radian = 1.f;

	return std::acos(radian);
}

