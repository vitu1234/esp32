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


const char *req =
	"{\"method\": \"digital_write\", "
	"\"params\": [\"users\", \"wheel\", \"audio\", \"video\"], \"rto\": \"/user/1234/out\"}";


int digital_write(int argc, JSMN_PARAMS_t argv)
{
    uint8_t i = 0;
    JSMN_PARAMS_t params = {{0}};
    JSMN_PARAMS_t expected_params = {"users", "wheel", "audio", "video"};

    memcpy(params, argv, sizeof(JSMN_PARAMS_t));
    TEST_ASSERT_EQUAL_STRING(expected_params[0], argv[0]);
    TEST_ASSERT_EQUAL_STRING(expected_params[1], argv[1]);
    TEST_ASSERT_EQUAL_STRING(expected_params[2], argv[2]);
    TEST_ASSERT_EQUAL_STRING(expected_params[3], argv[3]);

    return EXIT_SUCCESS;
}

int digital_read(int argc, JSMN_PARAMS_t argv)
{
    // just to illustrate having multiple functions
    return EXIT_SUCCESS;
}

TEST_CASE("erpc test case", "[erpc]")
{
    static const char *JSON_STRING = "{\"type\": \"bme280\", \"params\": [119]}";  // 119 = 0x77

    erpc_add_function("digital_write", digital_write);
    erpc_add_function("digital_read", digital_read);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, erpc_call(req) );
}
