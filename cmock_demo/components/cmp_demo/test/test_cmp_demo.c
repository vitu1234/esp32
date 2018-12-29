#include <stdio.h>
#include <stdint.h>

#include "unity.h"
#include "cmp_demo.h"

#include "mock_dep_demo.h"

TEST_CASE("test cmp_demo init", "[demo]")
{
    dep_init_ExpectAndReturn(42, 420);

    TEST_ASSERT_EQUAL_INT(420, cmp_init());
}
