#include <forsyde.hpp>
#include "top.hpp"

#ifdef FORSYDE_SELF_REPORTING
#include <fcntl.h>
#endif

int sc_main(int argc, char* argv[])
{
    top top1("top1");
    sc_core::sc_start();
    return 0;
}
