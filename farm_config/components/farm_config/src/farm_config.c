/**
 * Copyright 2018 Fink Labs GmbH
 * Additions Copyright 2018 Espressif Systems (Shanghai) PTE LTD
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


/* farm_config.c: Implementation of a mean function of testable component.
   See test/test_mean.c for the associated unit test.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <jsmn.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "farm_config.h"

#define JSON_FILE_PATH "test.json"
#define BUFFER_SIZE 5000
#define MAX_TOKEN_COUNT 128

int testable_mean(const int* values, int count)
{
    if (count == 0) {
        return 0;
    }
    int sum = 0;
    for (int i = 0; i < count; ++i) {
        sum += values[i];
    }
    return sum / count;
}

/*
// Read files
void readfile(char* filepath, char* fileContent)
{
    FILE *f;
    char c;
    int index = 0;
    f = fopen(filepath, "rt");
    while((c = fgetc(f)) != EOF){
        fileContent[index] = c;
        index += 1;
    }
    fileContent[index] = '\0';
}
*/

//static const char *JSON_STRING = "{\"name\" : \"Manash\", \"website\" : \"https://blog.manash.me\"}";

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}


struct Bme280 {
    int p1;
};

struct Apfel {
    char *name;
};

//int parseJSON(char *filepath, void callback(char *, char*)){
int json_parse_sense(const char *json_string)
{
    //char JSON_STRING[BUFFER_SIZE];

    //char value[1024];
    //char key[1024];

    //readfile(JSON_FILE_PATH, JSON_STRING);

   int i;
   int r;

   jsmn_parser p;
   jsmntok_t t[MAX_TOKEN_COUNT];

   jsmn_init(&p);

   r = jsmn_parse(&p, json_string, strlen(json_string), t, sizeof(t)/(sizeof(t[0])));
   if (r < 0) {
       printf("Failed to parse JSON: %d\n", r);
       return 1;
   }

   /* Assume the top-level element is an object */
   if (r < 1 || t[0].type != JSMN_OBJECT) {
       printf("Object expected\n");
       return 1;
   }

   /* Loop over all keys of the root object */
   for (i = 1; i < r; i++){
		if (jsoneq(json_string, &t[i], "type") == 0) {
			/* We may use strndup() to fetch string value */
			printf("- Name: %.*s\n", t[i+1].end-t[i+1].start,
					json_string + t[i+1].start);
			i++;
		} else {
			printf("Unexpected key: %.*s\n", t[i].end-t[i].start,
					json_string + t[i].start);
            return EXIT_FAILURE;
		}

       /*
       jsmntok_t json_value = t[i+1];
       jsmntok_t json_key = t[i];


       int string_length = json_value.end - json_value.start;
       int key_length = json_key.end - json_key.start;

       int idx;

       for (idx = 0; idx < string_length; idx++){
           value[idx] = JSON_STRING[json_value.start + idx ];
       }

       for (idx = 0; idx < key_length; idx++){
           key[idx] = JSON_STRING[json_key.start + idx];
       }

       value[string_length] = '\0';
       key[key_length] = '\0';

       //callback(key, value);
       printf("%s : %s\n", key, value);

       i++;
       */
   }

	return EXIT_SUCCESS;
}
