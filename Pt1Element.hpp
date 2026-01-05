#ifndef H_PT1ELEMENT
#define H_PT1ELEMENT

#include <cmath>
#include <concepts>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <iostream>

namespace bml {
	template <std::floating_point T>
	class Pt1Element {
	public:
		using value_type = T;

		constexpr Pt1Element(T K, T Tconst, T y0 = T{ 0 })
			: _K(K), _T(Tconst), _y(y0)
		{
			validate_params();
		}

		T Step(T dt, T target) {
			validate_step(dt, target);

			if (dt == T{ 0 }) return _y;

			const T a = std::exp(-dt / _T);
			_y = a * _y + (T{ 1 } - a) * (_K * target);

			return _y;
		}

		constexpr void Reset(T y0 = T{ 0 }) noexcept { _y = y0; }
		constexpr T Output() const noexcept { return _y; }

		void SetParams(T K, T Tconst) {
			_K = K;
			_T = Tconst;
			validate_params();
		}

	private:
		void validate_params() const {
			if (!std::isfinite(_K) || !std::isfinite(_T))
				throw std::invalid_argument("PT1: K and T must be finite.");
			if (!(_T > T{ 0 }))
				throw std::invalid_argument("PT1: time constant T must be > 0.");
		}

		void validate_step(T dt, T target) const {
			if (!std::isfinite(dt) || !std::isfinite(target))
				throw std::invalid_argument("PT1: dt and target must be finite.");
			if (dt < T{ 0 })
				throw std::invalid_argument("PT1: dt must be >= 0.");
		}

		T _K;
		T _T;
		T _y;
	};
}

#endif