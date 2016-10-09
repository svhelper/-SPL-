#ifndef __aux_if_hpp__
#define __aux_if_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace aux {

template <bool cond, class _TRUE_, class _FALSE_>
struct if_c							{ typedef _TRUE_  _result; };
template <class _TRUE_, class _FALSE_>
struct if_c<false, _TRUE_, _FALSE_>	{ typedef _FALSE_ _result; };

} // namespace aux
//////////////////////////////////////////////////////////////////////////
#endif /*__aux_if_hpp__*/
