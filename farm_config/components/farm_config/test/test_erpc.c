/**
 * Copyright 2018 Fink Labs GmbH
 * Additions Copyright 2018 Espressif Systems (Shanghai) PTE LTD
 * Additions Copyright 2017 Project Iota, Drasko Draskovic (https://github.com/drasko)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "erpc.h"
#include "unity.h"


typedef struct {
    int value
} demo_t;

int comp123_init(int argc, JSMN_PARAMS_t argv)
{
    // demo component init use
    JSMN_PARAMS_t params = {{0}};
    JSMN_PARAMS_t expected_params = {"119"};

    memcpy(params, argv, sizeof(JSMN_PARAMS_t));
    TEST_ASSERT_EQUAL_STRING(expected_params[0], argv[0]);

    demo_t *s1 = malloc(sizeof(demo_t));
    s1->value = 42;
    return s1;
}

int comp123_work(demo_t *inst)
{
    // demo function use; note: this has no parameters besides the struct!
    TEST_ASSERT_EQUAL_INT(42, inst->value);

    return EXIT_SUCCESS;
}

TEST_CASE("erpc test add component", "[erpc]")
{
    erpc_add_component("comp123", comp123_init);
}

TEST_CASE("erpc test add function", "[erpc]")
{
    erpc_add_function("comp123_do_work", comp123_work);
}

TEST_CASE("erpc test comp123_work", "[erpc]")
{
    demo_t s1;
    s1.value = 42;

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, comp123_work(&s1));
}

TEST_CASE("erpc test init component", "[erpc]")
{
    const char *config = "{"
        "\"name\": \"inside\", "
        "\"component\": \"comp123\", "
        "\"params\": [\"119\"] }";

    erpc_add_component("comp123", comp123_init);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, erpc_component_init(config));
}

TEST_CASE("erpc test call function", "[erpc]")
{
    const char *config = "{"
        "\"name\": \"inside\", "
        "\"component\": \"comp123\", "
        "\"params\": [\"119\"] }";

    erpc_add_component("comp123", comp123_init);
    erpc_add_function("comp123_work", comp123_work);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, erpc_component_init(config));
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, erpc_call("inside", "comp123_work"));
}
