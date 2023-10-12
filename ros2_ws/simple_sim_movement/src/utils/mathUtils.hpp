#ifndef MATHUTILS_HPP_
#define MATHUTILS_HPP_

namespace Utils
{
	/**
	 *
	 */
	const double PI = 3.141592654;

	/**
	 *
	 */
	class MathUtils
	{
		public:
			/**
			 *
			 */
			static double toRadians( double aDegrees);
			/**
			 *
			 */
			static double toDegrees( double aRadian);
	}; // class Math
}// namespace Utils
#endif // MATHUTILS_HPP_