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
#include <_aux_list.hpp>

//////////////////////////////////////////////////////////////////////////
#include <mcu_cfg.hpp>

//////////////////////////////////////////////////////////////////////////
#include <objtypes.hpp>
#include <gpio.hpp>
#include <dma.hpp>
#include <clock.hpp>
#include <uart.hpp>
#include <spi.hpp>
#include <adc.hpp>

//////////////////////////////////////////////////////////////////////////
// Include the main header for selected MCU
#include __METALLL_PORT(.hpp)

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {

template< typename sysclock, class modules >
class mcu_port;

template< typename sysclock, _VAR_ARGS_DEF( = dummy::obj ) >
class mcu : public obj::obj< obj::type_id::mcu >
{
private:
	mcu();
	~mcu();

public:
	typedef sysclock						_sysclock;
	typedef ::aux::list<_VAR_ARGS_LIST()>	_modules;

	typedef mcu_port<_sysclock, _modules>	_port_;

public:
	static void init()
	{
		_port_::init();
	}
	
	template< typename sysclock_new >
	static void switch_sysclock()
	{
		_port_::template switch_sysclock<sysclock_new>();
	}
};

} // namespace mcu


/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif /*__mcu_hpp__*/
