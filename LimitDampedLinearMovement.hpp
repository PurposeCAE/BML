#ifndef BML_LIMIT_DAMPED_LINEAR_MOVEMENT_HPP
#define BML_LIMIT_DAMPED_LINEAR_MOVEMENT_HPP

#include <concepts>
#include <stdexcept>
#include <cmath>

namespace bml {

	template <std::floating_point Time_t, std::floating_point Value_t>
	class LimitDampedLinearMovement {
	public:
		LimitDampedLinearMovement(Value_t stroke, Time_t duration, Value_t dampingStroke, Value_t current = 0)
			: MAX_ACCELERATION(determineMaxAcceleration(stroke, duration, dampingStroke))
			, MAX_VELOCITY(determineMaxVelocity(dampingStroke))
			, DAMPING_STROKE(dampingStroke)
			, _current(current)
		{ }

		Value_t Step(Time_t dt, Value_t target) noexcept {
			if (target == _current) [[likely]] {
				currentVelocity = Value_t{ 0 };
				return _current;
			}

			Value_t deviation = target - _current;
			Value_t absDeviation = std::abs(deviation);
			Value_t direction = deviation < Value_t{ 0 } ? Value_t{ -1.0 } : Value_t{ 1.0 };

			// Deceleration
			if (absDeviation <= DAMPING_STROKE) 
				currentVelocity -= direction * MAX_ACCELERATION * static_cast<Value_t>(dt);
			// Acceleration
			else 
				currentVelocity += direction * MAX_ACCELERATION * static_cast<Value_t>(dt);

			// Linear movement
			if (std::abs(currentVelocity) >= MAX_VELOCITY) {
				currentVelocity = MAX_VELOCITY * direction;
			}

			Value_t step = currentVelocity * static_cast<Value_t>(dt);

			// Target reached
			if (std::abs(step) >= absDeviation) {
				_current = target;
				currentVelocity = Value_t{ 0 };
			}

			// Euler
			else [[likely]] {
				_current += step;
			}

			return _current;
		}

		Value_t get_velocity() const noexcept {
			return currentVelocity;
		}

	private:
		const Value_t MAX_ACCELERATION;
		const Value_t MAX_VELOCITY;
		const Value_t DAMPING_STROKE;
		static void evaluateParameters(Value_t stroke, Time_t duration, Value_t dampingStroke) {
			if (stroke < Value_t{ 0 })
				throw std::invalid_argument("Stroke is less than 0!");
			if (duration <= Time_t{ 0 })
				throw std::invalid_argument("Duration is less or equal than 0!");
			if (dampingStroke <= Value_t{ 0 })
				throw std::invalid_argument("AccelerationTime is less or equal than 0!");
			if (dampingStroke * 2 > stroke)
				throw std::invalid_argument("DampingStroke is to big!");
		}
		static Value_t determineMaxAcceleration(Value_t stroke, Time_t duration, Value_t dampingStroke) {
			evaluateParameters(stroke, duration, dampingStroke);

			return (stroke * stroke + Value_t{ 4.0 } * dampingStroke + Value_t{ 4.0 } * dampingStroke * dampingStroke)
				/ (Value_t{ 2.0 } * dampingStroke * static_cast<Value_t>(duration) * static_cast<Value_t>(duration));
		}
		Value_t determineMaxVelocity(Value_t dampingStroke) const noexcept {
			return std::sqrt(Value_t{ 2.0 } * dampingStroke * MAX_ACCELERATION);
		}

		Value_t _current;
		Value_t currentVelocity = 0;
	};
}

#endif // BML_LIMIT_DAMPED_LINEAR_MOVEMENT_HPP