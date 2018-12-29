#include <stdio.h>
#include <string.h>

#include "dep_demo.h"
#include "cmp_demo.h"


int cmp_init(void) {
    int value = 42;
    return dep_init(value);
};
