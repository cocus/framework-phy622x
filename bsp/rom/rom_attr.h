#ifndef ROM_VARS_H
#define ROM_VARS_H

#ifdef USE_ROMSYM_ALIAS
#define ROM_NAME(name) _symrom_##name
#else
#define ROM_NAME _symrom_#
#endif

#define ATTR_ROM_VAR extern volatile

#define ATTR_ROM_CONST extern const

#define ATTR_ROM_FN

#endif /* ROM_VARS_H */
