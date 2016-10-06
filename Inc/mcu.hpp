#ifndef __mcu_hpp__
#define __mcu_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/



//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//////////////////////////////////////////////////////////////////////////
#include <objtypes.hpp>
#include <gpio.hpp>
#include <dma.hpp>
#include <clock.hpp>

//////////////////////////////////////////////////////////////////////////
#include <mcu_cfg.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {

template< typename sysclock, _VAR_ARGS_DEF( = dummy::obj ) >
class mcu : public obj::obj< objtype::mcu >
{
public:
};

} // namespace mcu


/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif /*__mcu_hpp__*/
