/* test_farm_config.c: Implementation of a testable component.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <limits.h>
#include "unity.h"
#include "farm_config.h"

#define countof(x) (sizeof(x)/sizeof(x[0]))

TEST_CASE("Mean of an empty array is zero", "[mean]")
{
    const int values[] = { 0 };
    TEST_ASSERT_EQUAL(0, testable_mean(values, 0));
}

TEST_CASE("Mean of a test vector", "[mean]")
{
    const int v[] = {1, 3, 5, 7, 9};
    TEST_ASSERT_EQUAL(5, testable_mean(v, countof(v)));
}

/* This test case currently fails, and developer has added a tag to indicate this.
 * For the test runner, "[fails]" string does not carry any special meaning.
 * However it can be used to filter out tests when running.
 */
TEST_CASE("Another test case which fails", "[mean][fails]")
{
    const int v1[] = {INT_MAX, INT_MAX, INT_MAX, INT_MAX};
    TEST_ASSERT_EQUAL(INT_MAX, testable_mean(v1, countof(v1)));
}

TEST_CASE("JSON test case", "[json]")
{
    static const char *JSON_STRING = "{\"type\": \"bme280\", \"params\": [119]}";  // 119 = 0x77

    //const int v1[] = {INT_MAX, INT_MAX, INT_MAX, INT_MAX};
    TEST_ASSERT_EQUAL(0, json_parse_sense(JSON_STRING) );
}