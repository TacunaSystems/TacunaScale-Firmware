/*
 * Unity build wrapper for scpi-parser library.
 * Compiles all upstream .c files in a single translation unit so PlatformIO
 * picks them up without needing a complex source_filter.
 */

#include "../../extern/scpi-parser/libscpi/src/error.c"
#include "../../extern/scpi-parser/libscpi/src/expression.c"
#include "../../extern/scpi-parser/libscpi/src/fifo.c"
#include "../../extern/scpi-parser/libscpi/src/ieee488.c"
#include "../../extern/scpi-parser/libscpi/src/lexer.c"
#include "../../extern/scpi-parser/libscpi/src/minimal.c"
#include "../../extern/scpi-parser/libscpi/src/parser.c"
#include "../../extern/scpi-parser/libscpi/src/units.c"
#include "../../extern/scpi-parser/libscpi/src/utils.c"
