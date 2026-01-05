#ifndef H_PT4_ELEMENT
#define H_PT4_ELEMENT

#include <cmath>
#include <concepts>

#include "Pt1Element.hpp"

namespace bml {
	template <std::floating_point T>
	class Pt4Element {
	public:
		/// <summary>
		/// Determines the time constant T based on the desired settle time and settle tolerance.
		/// </summary>
		/// <param name="K">Gain factor (take 1 as default)</param>
		/// <param name="settleTime">The time which is needed to reach the settleTolerance</param>
		/// <param name="settleTolerance">The tolerance (e.g. 0.99) the value has to be reached
		/// to be defined as settled</param>
		constexpr Pt4Element(T K, T settleTime, T settleTolerance)
			: _T(determineT(settleTime, settleTolerance))
			, x1(T{ 0 }), x2(T{ 0 }), x3(T{ 0 }), x4(T{ 0 })
		{
		}

		constexpr T Step(T dt, T target) {
			T dx1 = (target - x1) / _T;
			T dx2 = (x1 - x2) / _T;
			T dx3 = (x2 - x3) / _T;
			T dx4 = (x3 - x4) / _T;

			x1 += dx1 * dt;
			x2 += dx2 * dt;
			x3 += dx3 * dt;
			x4 += dx4 * dt;

			return x4;
		}

	private:
		const T _T;
		T x1;
		T x2;
		T x3;
		T x4;
		T determineT(T settleTime, T settleTolerance) {
			const size_t maxIterations = 100;
			const T toleranceForT = std::numeric_limits<T>::epsilon();
			T Tvalue = solveX(settleTolerance, maxIterations, toleranceForT);
			return settleTime / Tvalue;
		}
		T solveX(T settleTolerance, size_t maxIterations, T toleranceForT) {

			T epsilon = T{ 1 } - settleTolerance;

			T low = T{ 0 };
			T high = T{ 10 };

			while (function(high, epsilon) > T{ 0 })
			{
				high *= T{ 2 };
				if (high > T{ 1e6 }) {
					throw std::runtime_error("Pt4Element: Cannot determine T for given settle tolerance.");
				}
			}

			for (size_t i = 0; i < maxIterations; i++)
			{
				T mid = (low + high) / T{ 2 };
				T value = function(mid, epsilon);
				if (std::abs(value) < toleranceForT) {
					return mid;
				}
				if (value > T{ 0 }) {
					low = mid;
				}
				else {
					high = mid;
				}
			}
			return (low + high) / T{ 2 };
		}
		T function(T x, T epsilon) {
			T polynom = T{ 1 } + x + T{ 0.5 }*x * x + T{ 1 } / T{ 6 }*x * x * x;
			return std::exp(-x) * polynom - epsilon;
		}
	};
}

#endif // !H_PT4_ELEMENT
