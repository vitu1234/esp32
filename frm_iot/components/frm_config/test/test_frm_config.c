/**
 * Copyright 2018, 2019 Fink Labs GmbH
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

#include "frm_config.h"
#include "jsmn.h"
#include "unity.h"


typedef struct {
    int value
} demo_t;

int comp123_init(int argc, frm_params_type argv)
{
    // demo component init use
    frm_params_type params = {{0}};
    frm_params_type expected_params = {"inside", "119"};

    memcpy(params, argv, sizeof(frm_params_type));
    TEST_ASSERT_EQUAL_STRING(expected_params[0], argv[0]);
    TEST_ASSERT_EQUAL_STRING(expected_params[1], argv[1]);

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

TEST_CASE("test frm_config add component", "[frm_config]")
{
    frm_config_add_component("comp123", comp123_init);
}

TEST_CASE("test frm_config add function", "[frm_config]")
{
    frm_config_add_function("comp123_do_work", comp123_work);
}

TEST_CASE("test frm_config comp123_work", "[frm_config]")
{
    demo_t s1;
    s1.value = 42;

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, comp123_work(&s1));
}

TEST_CASE("test frm_config init component", "[frm_config]")
{
    const char *config = "{"
        "\"name\": \"inside\", "
        "\"component\": \"comp123\", "
        "\"params\": [\"119\"] }";

    int i = 0;
	int r;
	jsmn_parser p;
	jsmntok_t tokens[128]; /* We expect no more than 128 tokens */

    // parsed the string into tokens using jsmn
	jsmn_init(&p);
	r = jsmn_parse(&p, config, strlen(config), tokens, sizeof(tokens)/sizeof(tokens[0]));
	if (r < 0)
    {
		printf("Failed to parse JSON: %d\n", r);
        TEST_FAIL();
	}

	// in order to gain a deeper understanding it is important
	// to understand what a token is!!
	// => a token is
    //   * a struct containing a type and
    //   * number of child-nodes (JSMN_OBJECT, JSMN_ARRAY)
    //   * or start and end (JSMN_STRING)
    // print token types and sizes of tokens information
    //static char* token_types[] = { "JSMN_UNDEFINED", "JSMN_OBJECT", "JSMN_ARRAY", "JSMN_STRING", "JSMN_PRIMITIVE" };
    //for(uint x=0; x < r; x++)
    //{
    //    printf("token%u type: %s\n", x, token_types[(uint)(&tokens[x])->type]);
    //    printf("token%u size: %u\n", x, (uint)(&tokens[x])->size);
    //}

	// we assume the top-level element is an object
	if (r < 1 || tokens[0].type != JSMN_OBJECT)
    {
		printf("Object expected\n");
		return EXIT_FAILURE;
	}

    frm_config_add_component("comp123", comp123_init);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, frm_config_component_init(config, &i, tokens));
    TEST_ASSERT_EQUAL_INT(8, i);
}

TEST_CASE("test frm_config call", "[frm_config]")
{
    const char *config = "{"
        "\"name\": \"inside\", "
        "\"component\": \"comp123\", "
        "\"params\": [\"119\"] }";

    int i = 0;
	int r;
	jsmn_parser p;
	jsmntok_t tokens[128]; /* We expect no more than 128 tokens */

    // parsed the string into tokens using jsmn
	jsmn_init(&p);
	r = jsmn_parse(&p, config, strlen(config), tokens, sizeof(tokens)/sizeof(tokens[0]));
	if (r < 0)
    {
		printf("Failed to parse JSON: %d\n", r);
        TEST_FAIL();
	}

	// we assume the top-level element is an object
	if (r < 1 || tokens[0].type != JSMN_OBJECT)
    {
		printf("Object expected\n");
		return EXIT_FAILURE;
	}

    frm_config_add_component("comp123", comp123_init);
    frm_config_add_function("comp123_work", comp123_work);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, frm_config_component_init(config, &i, tokens));
    TEST_ASSERT_EQUAL_INT(8, i);
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, frm_config_call("inside", "comp123_work"));
}
