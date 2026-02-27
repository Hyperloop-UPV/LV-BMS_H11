#ifndef IP_BITMASK_H
#define IP_BITMASK_H

#define IP_BITMASK_BITS 24

#if IP_BITMASK_BITS >= 24 && IP_BITMASK_BITS <= 32
# define IP_BITMASK_PREFIX "255.255.255."
# define IP_BITMASK_POSTFIX ""
# define IP_BITMASK_BYTE_BIT (IP_BITMASK_BITS - 24)
#elif IP_BITMASK_BITS >= 16
# define IP_BITMASK_PREFIX "255.255."
# define IP_BITMASK_POSTFIX ".0"
# define IP_BITMASK_BYTE_BIT (IP_BITMASK_BITS - 16)
#elif IP_BITMASK_BITS >= 8
# define IP_BITMASK_PREFIX "255."
# define IP_BITMASK_POSTFIX ".0.0"
# define IP_BITMASK_BYTE_BIT (IP_BITMASK_BITS - 8)
#elif IP_BITMASK_BITS >= 0
# define IP_BITMASK_PREFIX ""
# define IP_BITMASK_POSTFIX ".0.0.0"
# define IP_BITMASK_BYTE_BIT (IP_BITMASK_BITS)
#else
# error Invalid Ip bitmask bits, must be in range [0,32]
#endif

#if IP_BITMASK_BYTE_BIT == 0
# define IP_BITMASK_BYTE_CSTR "0"
#elif IP_BITMASK_BYTE_BIT == 1
# define IP_BITMASK_BYTE_CSTR "1"
#elif IP_BITMASK_BYTE_BIT == 2
# define IP_BITMASK_BYTE_CSTR "3"
#elif IP_BITMASK_BYTE_BIT == 3
# define IP_BITMASK_BYTE_CSTR "7"
#elif IP_BITMASK_BYTE_BIT == 4
# define IP_BITMASK_BYTE_CSTR "15"
#elif IP_BITMASK_BYTE_BIT == 5
# define IP_BITMASK_BYTE_CSTR "31"
#elif IP_BITMASK_BYTE_BIT == 6
# define IP_BITMASK_BYTE_CSTR "63"
#elif IP_BITMASK_BYTE_BIT == 7
# define IP_BITMASK_BYTE_CSTR "127"
#elif IP_BITMASK_BYTE_BIT == 8
# define IP_BITMASK_BYTE_CSTR "255"
#else
# error Invalid bitmask byte bit
#endif

#define IP_BITMASK_CSTR \
    IP_BITMASK_PREFIX \
    IP_BITMASK_BYTE_CSTR \
    IP_BITMASK_POSTFIX

#endif // IP_BITMASK_H