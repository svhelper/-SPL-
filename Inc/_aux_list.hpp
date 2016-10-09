#ifndef __aux_list_hpp__
#define __aux_list_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

#include <objtypes.hpp>
#include <_aux_if.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace aux {

#define LIST_GET_NEXT(CUR)		typename if_c< ::mcu::obj::eq::template check<typename CUR::_next, ::mcu::dummy::eol>::_result, ::mcu::dummy::eol, typename CUR::_next >::_result

//////////////////////////////////////////////////////////////////////////
template < class head, class tail >
class list_item : public head
{
public:
	typedef head _cur;
	typedef tail _next;
};

//////////////////////////////////////////////////////////////////////////
template <class CUR, class FIND, class CMP>
class list_find_c
{
public:
	typedef typename if_c
		<
			CMP::template check<CUR, FIND>::_result,
			CUR,
			typename list_find_c< LIST_GET_NEXT(CUR), FIND, CMP >::_result
		>::_result _result;
};

//------------------------------------------------------------------------
template <class FIND, class CMP >
class list_find_c< ::mcu::dummy::eol, FIND, CMP >
{
public:
	typedef ::mcu::dummy::invalid _result;

//	STATIC_ASSERT(false, "The item was not found!");
};

//------------------------------------------------------------------------
template <class CUR, class FIND, class CMP>
class list_find
{
public:
	typedef typename list_find_c<CUR, FIND, CMP>::_result _result;
};

//////////////////////////////////////////////////////////////////////////
template <class CUR, class CALL_BACK >
class list_traverse_c
{
public:
	typedef typename CALL_BACK::template _cb
		<
			CUR,
			typename list_traverse_c< LIST_GET_NEXT(CUR), CALL_BACK >::_result
		>::_result _result;
};

//------------------------------------------------------------------------
template < class CALL_BACK >
class list_traverse_c< ::mcu::dummy::eol, CALL_BACK >
{
public:
	typedef typename CALL_BACK::template _cb< ::mcu::dummy::eol, ::mcu::dummy::eol >::_result _result;

//	STATIC_ASSERT(false, "The item was not found!");
};

//------------------------------------------------------------------------
template <class CUR, class CALL_BACK >
class list_traverse
{
public:
	typedef typename list_traverse_c<CUR, CALL_BACK>::_result _result;
};

//////////////////////////////////////////////////////////////////////////
template < _VAR_ARGS_DEF() >
class list
	: public list_item < p00, list< _VAR_ARGS_LIST_SHIFT(), ::mcu::dummy::eol > >
{
	typedef list_item < p00, list< _VAR_ARGS_LIST_SHIFT(), ::mcu::dummy::eol > > list_items;
	
public:
	template <class T>
	class find
	{
	public:
		typedef typename list_find<list_items, T, ::mcu::obj::eq>::_result _result;
	};

	template < class CALL_BACK >
	class traverse
	{
	public:
		typedef typename list_traverse<list_items, CALL_BACK>::_result _result;
	};
};

} // namespace aux
//////////////////////////////////////////////////////////////////////////
#endif /*__aux_list_hpp__*/
