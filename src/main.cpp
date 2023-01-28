#include "common.h"
#include "scanRegistration/scanRegistration.h"

int main() {
    srOption option("VLP16", true);
    scanRegistration sr(option);
    return 0;
}
