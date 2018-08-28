#ifndef __CZ_COMMON_CZDEF_H__
#define __CZ_COMMON_CZDEF_H__

#ifndef CZ_EXTERN_C 
#  ifdef WIN_VS
#define CZ_EXTERN_C extern "C" __declspec(dllexport)
#  else
#    define CZ_EXTERN_C
#  endif
#endif

#ifndef CZ_EXPORTS
#  ifdef WIN_VS
#    define CZ_EXPORTS __declspec(dllexport)
#  else 
#    define CZ_EXPORTS
#  endif
#endif

#endif