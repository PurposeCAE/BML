#ifndef BML_LIMIT_DAMPED_LINEAR_MOVEMENT_HPP
#define BML_LIMIT_DAMPED_LINEAR_MOVEMENT_HPP

#include <concepts>
#include <stdexcept>
#include <cmath>

namespace bml {

	template <std::floating_point Time_t, std::floating_point Value_t>
	class LimitDampedLinearMovement {
	public:
		constexpr LimitDampedLinearMovement(Value_t stroke, Time_t duration, Time_t accelerationTime, Value_t current = 0)
			: MAX_ACCELERATION(determineMaxAcceleration(stroke, duration, accelerationTime))
			, MAX_VELOCITY(determineMaxVelocity(accelerationTime))
			, ACCELERATION_WIDTH(determineAccelerationWidth(accelerationTime))
			, _current(current)
		{ }

		constexpr Value_t Step(Time_t dt, Value_t target) {
			Value_t deviation = target - _current;
			Value_t absDeviation = std::abs(deviation);
			Value_t direction = deviation < Value_t{ 0 } ? Value_t{ -1.0 } : Value_t{ 1.0 };
			
			// Deceleration
			if (absDeviation <= ACCELERATION_WIDTH) {
				currentVelocity -= direction * MAX_ACCELERATION * static_cast<Value_t>(dt);
			}

			// Acceleration
			else {
				currentVelocity += direction * MAX_ACCELERATION * static_cast<Value_t>(dt);
			}

			// Linear movement
			if (std::abs(currentVelocity) >= MAX_VELOCITY) {
				currentVelocity = MAX_VELOCITY * direction;
			}

			// Euler
			_current += currentVelocity * static_cast<Value_t>(dt);

			// Target reached
			if (deviation <= static_cast<Value_t>(dt) * MAX_VELOCITY) {
				_current = target;
				currentVelocity = Value_t{ 0 };
			}

			return _current;
		}

	private:
		const Value_t MAX_ACCELERATION;
		const Value_t MAX_VELOCITY;
		const Value_t ACCELERATION_WIDTH;
		static void evaluateParameters(Value_t stroke, Time_t duration, Time_t accelerationTime) {
			if (stroke < Value_t{ 0 })
				throw std::invalid_argument("Stroke is less than 0!");
			if (duration < Time_t{ 0 })
				throw std::invalid_argument("Duration is less than 0!");
			if (accelerationTime < Time_t{ 0 })
				throw std::invalid_argument("AccelerationTime is less than 0!");

			Value_t t_acc = static_cast<Value_t>(accelerationTime);
			if ((t_acc * t_acc - 2 * t_acc + static_cast<Value_t>(duration)) == Value_t{ 0 })
				throw std::invalid_argument("Parameter constillation would result in division by zero!");
		}
		static Value_t determineMaxAcceleration(Value_t stroke, Time_t duration, Time_t accelerationTime) {
			evaluateParameters(stroke, duration, accelerationTime);

			Value_t t_acc = static_cast<Value_t>(accelerationTime);

			return (stroke - 2 * t_acc * t_acc - static_cast<Value_t>(duration) * t_acc)
				/ (t_acc * t_acc - 2 * t_acc + duration);
		}
		constexpr Value_t determineMaxVelocity(Time_t accelerationTime) const noexcept {
			return MAX_ACCELERATION * static_cast<Value_t>(accelerationTime);
		}
		constexpr Value_t determineAccelerationWidth(Time_t accelerationTime) const noexcept {
			return Value_t{ 0.5 }
				* MAX_ACCELERATION
				* static_cast<Value_t> (accelerationTime)
				* static_cast<Value_t> (accelerationTime);
		}

		Value_t _current;
		Value_t currentVelocity;
	};
}

#endif // BML_LIMIT_DAMPED_LINEAR_MOVEMENT_HPP