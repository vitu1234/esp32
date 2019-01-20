/* Example test application for jsmn.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "unity.h"
#include "jsmn.h"

#include "esp_log.h"
static const char *TAG = "FRM_JSMN";

static void print_banner(const char* text);

void app_main()
{
    /* These are the different ways of running registered tests.
     * In practice, only one of them is usually needed.
     *
     * UNITY_BEGIN() and UNITY_END() calls tell Unity to print a summary
     * (number of tests executed/failed/ignored) of tests executed between these calls.
     */
    print_banner("Executing one test by its name");
    UNITY_BEGIN();
    unity_run_test_by_name("Mean of an empty array is zero");
    UNITY_END();

    print_banner("Running tests with [mean] tag");
    UNITY_BEGIN();
    unity_run_tests_by_tag("[mean]", false);
    UNITY_END();

    print_banner("Running tests without [fails] tag");
    UNITY_BEGIN();
    unity_run_tests_by_tag("[fails]", true);
    UNITY_END();

    print_banner("Running all the registered tests");
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();

    print_banner("Starting interactive test menu");
    /* This function will not return, and will be busy waiting for UART input.
     * Make sure that task watchdog is disabled if you use this function.
     */
    unity_run_menu();
}

static void print_banner(const char* text)
{
    printf("\n#### %s #####\n\n", text);
}


// sample Json data to parse
static const char *JSON_STRING =
	"{"
      "\"sense\": ["
        "{"
          "\"name\": \"inside\","
          "\"component\": \"bme280\""
        "},"
        "{"
          "\"name\": \"outside\","
          "\"component\": \"bme280\","
          "\"params\": [\"119\", \"120\"]"
        "}"
      "],"
      "\"user\": \"mark\","
      "\"unex\": \"pected\""
    "}";


/*Compares up to (t-end-t->start) characters of the string s with  string js.*/
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}


/// parse sense object and return sense instance
///
/// @param i (int*) - number of tokens that already have been parsed
/// @param tokens (jsmntok_t*) - tokens parsed by jsmn
/// @return EXIT_SUCCESS / EXIT_FAILURE
static int parse_sense_object(int* i, jsmntok_t* tokens)
{
    // <== from HERE
    // assert the element is an object
    if (tokens[(*i)].type != JSMN_OBJECT)
    {
        printf("Object expected\n");
        //printf("got: %s\n", token_types[(uint)(&tokens[(*i)])->type]);
        return EXIT_FAILURE;
    }
    int object_size = tokens[(*i)].size;
    printf("  o Instance:\n");
    printf("    size: %u\n", tokens[(*i)].size);
    (*i)++;  // processed object token
    for (int k = 0; k < object_size; k++)  // loop over objects key-value pairs
    {
        printf("    i: %u; k: %u\n", (*i), k);
        if (jsoneq(JSON_STRING, &tokens[(*i)], "name") == 0)
        {
            (*i)++;  // processed key token
            // We may use strndup() to fetch string value
            printf("    Name: %.*s\n", tokens[(*i)].end-tokens[(*i)].start,
                    JSON_STRING + tokens[(*i)].start);
        } 
        else if (jsoneq(JSON_STRING, &tokens[(*i)], "component") == 0)
        {
            (*i)++;  // processed key token
            // We may use strndup() to fetch string value
            printf("    Component: %.*s\n", tokens[(*i)].end-tokens[(*i)].start,
                    JSON_STRING + tokens[(*i)].start);
        }
        else if (jsoneq(JSON_STRING, &tokens[(*i)], "params") == 0)
        {
            (*i)++;  // processed key token
            // we expect params to be an array of objects
			if (tokens[(*i)].type != JSMN_ARRAY)
            {
        		printf("    Params array expected\n");
                return EXIT_FAILURE;
			}
            int array_size = tokens[(*i)].size;
			printf("    Params:\n");
            printf("    size: %u\n", array_size);
            (*i)++;  // processed array token
			for (int l = 0; l < array_size; l++)  // loop over array
            {
				printf("    * %.*s\n", tokens[(*i)].end - tokens[(*i)].start, JSON_STRING + tokens[(*i)].start);
                (*i)++;  // processed value
            }
        }
        else
        {
            printf("XUnexpected key: %.*s\n", tokens[(*i)].end-tokens[(*i)].start,
                    JSON_STRING + tokens[(*i)].start);
        }
        (*i)++; // processed value token
    }
    // <== to HERE

    return EXIT_SUCCESS;
}


TEST_CASE("jsmn demo split parser into smaller parts test", "[jsmn]")
{
    int i = 0;
	int r;
	jsmn_parser p;
	jsmntok_t tokens[128]; /* We expect no more than 128 tokens */

    // parsed the string into tokens using jsmn
	jsmn_init(&p);
	r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), tokens, sizeof(tokens)/sizeof(tokens[0]));
	if (r < 0)
    {
		printf("Failed to parse JSON: %d\n", r);
		return EXIT_FAILURE;
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

	// Here we assume the top-level element is an object
	if (r < 1 || tokens[0].type != JSMN_OBJECT)
    {
		printf("Object expected\n");
		return EXIT_FAILURE;
	}

    printf("root_size: %u\n", r);
	// process all keys of the root object
	for (i = 1; i < r; i++) {  // start at 1 (processed root element)
		if (jsoneq(JSON_STRING, &tokens[i], "sense") == 0)
        {
            i++;  // processed key token
            // we expect sense to be an array of objects
			if (tokens[i].type != JSMN_ARRAY)
            {
        		printf("Array expected\n");
                return EXIT_FAILURE;
			}
            int array_size = tokens[i].size;
			printf("- Sense:\n");
            printf("  size: %u\n", array_size);
            i++;  // processed array token
			for (int j = 0; j < array_size; j++)  // loop over array
            {
			    // note how we moved the following code into parse_sense_object
                // <== from HERE
                /*
                // assert the element is an object
                if (tokens[i].type != JSMN_OBJECT) {
                    printf("Object expected\n");
                    printf("got: %s\n", token_types[(uint)(&tokens[i])->type]);
                    return EXIT_FAILURE;
                }
                int object_size = tokens[i].size;
                printf("  o Instance:\n");
                printf("    size: %u\n", tokens[i].size);
                i++;  // processed object token
                for (int k = 0; k < object_size; k++) {  // loop over objects key-value pairs
                    printf("    i: %u; k: %u\n", i, k);
                    if (jsoneq(JSON_STRING, &tokens[i], "name") == 0) {
                        i++;  // processed key token
                        // We may use strndup() to fetch string value
                        printf("    Name: %.*s\n", tokens[i].end-tokens[i].start,
                                JSON_STRING + tokens[i].start);
                    } else if (jsoneq(JSON_STRING, &tokens[i], "component") == 0) {
                        i++;  // processed key token
                        // We may use strndup() to fetch string value
                        printf("    Component: %.*s\n", tokens[i].end-tokens[i].start,
                                JSON_STRING + tokens[i].start);
                    // TODO param!
                    // TODO copy the string into the struct! probably strcpy... OR manage the memory!
                    } else {
                        printf("XUnexpected key: %.*s\n", tokens[i].end-tokens[i].start,
                                JSON_STRING + tokens[i].start);
                    }
                    i++; // processed value token
                }
                */
                // <== to HERE
                parse_sense_object(&i, tokens);
                //parse_actor_object(&i, tokens);
                //...
			}
		} 
        else if (jsoneq(JSON_STRING, &tokens[i], "user") == 0)
        {
			// We may use strndup() to fetch string value
			printf("- User: %.*s\n", tokens[i+1].end-tokens[i+1].start,
					JSON_STRING + tokens[i+1].start);
            i++; // processed value token
		} 
        else 
        {
			printf("Unexpected key: %.*s\n", tokens[i].end-tokens[i].start,
					JSON_STRING + tokens[i].start);
		}
	}

    TEST_ASSERT_EQUAL_INT(i, 21);
}
