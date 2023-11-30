// You should create a copy of this file in your flash loader project
// and configure it as described below

// when this macro is non-zero, your FlashInit function should accept
// extra 'argc' and 'argv' arguments as specified by the function
// prototype in 'flash_loader.h'
#define USE_ARGC_ARGV 0

// You can customize the memory reserved for passing arguments to FlashInit
// through argc and argv.
#if USE_ARGC_ARGV
// This specifies the maximum allowed number of arguments in argv
#define MAX_ARGS 5
// This specifies the maximum combined size of the arguments, including
// a trailing null for each argument
#define MAX_ARG_SIZE 64
#endif
