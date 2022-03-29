#ifndef _COMMON_INC_COMMON_MACROS_H__
#define _COMMON_INC_COMMON_MACROS_H__
#define DECLARE_BACKWARD       \
  namespace backward {         \
  backward::SignalHandling sh; \
  }
#endif