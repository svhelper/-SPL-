#ifndef STATIC_ASSERT
#	define ASSERT_CONCAT2(a, b)			a##b
#	define ASSERT_CONCAT1(a, b)			ASSERT_CONCAT2(a, b)
#	define ASSERT_CONCAT(a, b)			ASSERT_CONCAT1(a, b)
#	define STATIC_ASSERT(expr, msg)		enum { ASSERT_CONCAT(check_at_, __LINE__) = 1/(int)(!!(expr)) }
#
#	define FAIL_IF(expr)				!(expr)
#endif /*STATIC_ASSERT*/
