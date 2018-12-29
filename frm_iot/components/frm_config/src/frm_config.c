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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "jsmn.h"
#include "frm_config.h"

#define FUNC_TABLE_SIZE 1024
#define COMP_TABLE_SIZE 1024
#define INST_TABLE_SIZE 1024

#define ERPC_COMPONENT_KEY "component"
#define ERPC_NAME_KEY "name"
#define ERPC_PARAMS_KEY "params"
#define ERPC_NULL "null"


/**
 * Fowler/Noll/Vo (FNV) hash function, variant 1a
 */
static size_t fnv1a_hash(const unsigned char* cp)
{
    size_t hash = 0x811c9dc5;
    while (*cp) {
        hash ^= (unsigned char) *cp++;
        hash *= 0x01000193;
    }
    return hash;
}

/**
 * Helper function to compare strings (from jsmn)
 */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
            strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}


/**
 * comp_table is an array of function pointers which usually are init functions of components.
 * Function pointers are defined by the user.
 */
int (*comp_table[COMP_TABLE_SIZE])(int argc, frm_params_type argv) = {NULL};


/**
 * inst_table is an array of pointers. Each pointer points to the result from calling the init
 * function of a component.
 */
void *inst_table[INST_TABLE_SIZE] = {NULL};


/**
 * func_table is an array of function pointers which usually are init functions of components.
 * Function pointers are defined by the user.
 */
int (*func_table[FUNC_TABLE_SIZE])(void *) = {NULL};



/**
 * Add a component:
 *Populate the comp_table by using epc_add_component to add all components.
 *
 * Erpc is platform agnostic - it knows only to parse JSON and call the init function of 
 * a component with name `comp_idx` with the correct parameters. 
 * The actual component functionality is defined with the components init function.
 */
void frm_config_add_component(char* comp_name, void (*f)(int argc, frm_params_type argv))
{
    unsigned char comp_idx = fnv1a_hash((const unsigned char *)comp_name) % COMP_TABLE_SIZE;
    comp_table[comp_idx] = f;
}


/**
 * Add a function:
 * Populate the func_table by using epc_add_function to add all functions.
 *
 * The actual component functionality is defined with the components functions.
 */
void frm_config_add_function(char* func_name, void (*f)(void * inst))
{
    unsigned char func_idx = fnv1a_hash((const unsigned char *)func_name) % FUNC_TABLE_SIZE;
    func_table[func_idx] = f;
}


/**
 * Parse JSON and call adequate component init function from lookup table
 * Store the result of the init function call in inst_table[<name>]
 */
int frm_config_component_init(const char* config)
{
    int i;
    int r;
    jsmn_parser p;
    jsmntok_t t[128]; // We expect no more than 128 tokens
    frm_params_type params = {{0}};
    static unsigned char paramNb = 0;
    //static unsigned char name[16] = {0};
    static unsigned char component[16] = {0};
    static unsigned char method[16] = {0};
    static unsigned char inst_idx = 0;
    static unsigned char comp_idx = 0;

    jsmn_init(&p);

    r = jsmn_parse(&p, config, strlen(config), t, sizeof(t)/sizeof(t[0]));
    if (r < 0) {
        printf("Failed to parse JSON: %d\n", r);
        return 1;
    }

    // Assume the top-level element is an object
    if (r < 1 || t[0].type != JSMN_OBJECT) {
        printf("Object expected\n");
        return 1;
    }

    // Loop over all keys of the root object
    for (i = 1; i < r; i++) {
        if (jsoneq(config, &t[i], ERPC_NAME_KEY) == 0) {
            int size = t[i+1].end-t[i+1].start;
        
            // We may use strndup() to fetch string value
            memcpy(params[0], config + t[i+1].start, t[i+1].end-t[i+1].start);
            params[0][size] = '\0';

            inst_idx = fnv1a_hash(params[0]) % INST_TABLE_SIZE;
            i++;
        } else if (jsoneq(config, &t[i], ERPC_COMPONENT_KEY) == 0) {
            int size = t[i+1].end-t[i+1].start;
        
            // We may use strndup() to fetch string value
            memcpy(component, config + t[i+1].start, t[i+1].end-t[i+1].start);
            component[size] = '\0';

            comp_idx = fnv1a_hash(component) % COMP_TABLE_SIZE;
            i++;
        } else if (jsoneq(config, &t[i], ERPC_PARAMS_KEY) == 0) {
            int j;

            //printf("- Params:\n");
            // Reset paramNb
            paramNb = 1;  // name is 1st parameter

            if (t[i+1].type != JSMN_ARRAY) {
                continue; // We expect params to be an array of strings
            }
            for (j = 0; j < t[i+1].size; j++) {
                jsmntok_t *g = &t[i+j+2];
                int paramSize = g->end - g->start;
                memcpy(params[paramNb], config + g->start, paramSize);
                params[paramNb][paramSize] = '\0';

                //printf("  * %s\n", params[paramNb]);
                paramNb++;
            }
            i += t[i+1].size + 1;
        } else {
            printf("Unexpected key: %.*s\n", t[i].end-t[i].start,
            config + t[i].start);
        }
    }

    // Call the components init function and store the response as instance in inst_table
    //voiddevice = 
    inst_table[inst_idx] = comp_table[comp_idx](paramNb, params);
    return EXIT_SUCCESS;
}


/**
 * call instance with adequate function from lookup tables
 */
int frm_config_call(const char* inst, const char* func)
{
    // lookup instance and function
    unsigned char inst_idx = fnv1a_hash((const unsigned char *)inst) % INST_TABLE_SIZE;
    unsigned char func_idx = fnv1a_hash((const unsigned char *)func) % FUNC_TABLE_SIZE;

    // call function with instance
    return func_table[func_idx](inst_table[inst_idx]);
}
