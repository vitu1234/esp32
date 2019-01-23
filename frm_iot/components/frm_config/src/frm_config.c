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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "jsmn.h"
#include "frm_config.h"

#include "esp_log.h"
static const char *TAG = "FRM_CONFIG";

#define FUNC_TABLE_SIZE 1024
#define COMP_TABLE_SIZE 1024
#define INST_TABLE_SIZE 1024

#define ERPC_COMPONENT_KEY "component"
#define ERPC_NAME_KEY "name"
#define ERPC_PARAMS_KEY "params"
#define ERPC_NULL "null"


/// Fowler/Noll/Vo (FNV) hash function, variant 1a
///
/// @param cp (const unsigned char*) - name of the component
/// @return hash
static size_t fnv1a_hash(const unsigned char* cp)
{
    size_t hash = 0x811c9dc5;
    while (*cp) {
        hash ^= (unsigned char) *cp++;
        hash *= 0x01000193;
    }
    return hash;
}

/// Helper function to compare strings (from jsmn)
///
/// @param json (const char *) - json configuration
/// @param tok (jsmntok_t *) - token
/// @param s (const char *) - string to compare to
/// @return 0 match / -1 no match
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
            strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}


/// comp_table is an array of function pointers which usually are init functions of components.
/// Function pointers are defined by the user.
int (*comp_table[COMP_TABLE_SIZE])(int argc, frm_params_type argv) = {NULL};


/// inst_table is an array of pointers. Each pointer points to the result from calling the init
/// function of a component.
void *inst_table[INST_TABLE_SIZE] = {NULL};


/// func_table is an array of function pointers which usually are functions of components.
/// Function pointers are defined by the user.
int (*func_table[FUNC_TABLE_SIZE])(void *, int argc, frm_params_type argv) = {NULL};


/// Add a component
///
/// Populate the comp_table by using epc_add_component to add all components.
/// frm_config knows only to parse JSON and call the init function of 
/// a component with name `comp_idx` with the correct parameters. 
/// The actual component functionality is defined with the components init function.
///
/// @param comp_name (char*) - name of the component
/// @param f (void *) - init function of the component
void frm_config_add_component(char* comp_name, void (*f)(int argc, frm_params_type argv))
{
    unsigned char comp_idx = fnv1a_hash((const unsigned char *)comp_name) % COMP_TABLE_SIZE;
    comp_table[comp_idx] = f;
}


/// Add a function:
///
/// Populate the func_table by using frm_config_add_function to add all functions.
/// @param func_name (char*) - name of the function
/// @param f (void*) - function
void frm_config_add_function(char* func_name, void (*f)(void * inst, int argc, frm_params_type argv))
{
    unsigned char func_idx = fnv1a_hash((const unsigned char *)func_name) % FUNC_TABLE_SIZE;
    func_table[func_idx] = f;
}


/// Parse JSON and call adequate component init function from lookup table
/// Store the result of the init function call in inst_table[<name>]
///
/// @param config (char*) - config to read the values
/// @param i (int*) - number of tokens that already have been parsed
/// @param tokens (jsmntok_t*) - tokens parsed by jsmn
/// @return EXIT_SUCCESS / EXIT_FAILURE
int frm_config_component_init(const char* config, int *i, jsmntok_t *tokens)
{
    frm_params_type params = {{0}};
    static unsigned char params_id = 0;
    static unsigned char component[16] = {0};
    static unsigned char method[16] = {0};
    static unsigned char inst_idx = 0;
    static unsigned char comp_idx = 0;

  // assert token is an object
  if (tokens[(*i)].type != JSMN_OBJECT)
  {
    ESP_LOGE(TAG, "Object expected");
    return EXIT_FAILURE;
  }
  int object_size = tokens[(*i)].size;
  (*i)++;  // processed object token
  for (int k = 0; k < object_size; k++)  // loop over objects key-value pairs
    {
        if (jsoneq(config, &tokens[(*i)], ERPC_NAME_KEY) == 0) 
        {
            (*i)++;  // processed key token
            int size = tokens[(*i)].end-tokens[(*i)].start;
            // We may use strndup() to fetch string value
            memcpy(params[0], config + tokens[(*i)].start, size);
            params[0][size] = '\0';

            inst_idx = fnv1a_hash(params[0]) % INST_TABLE_SIZE;
            (*i)++;
        }
        else if (jsoneq(config, &tokens[(*i)], ERPC_COMPONENT_KEY) == 0)
        {
            (*i)++;  // processed key token
            int size = tokens[(*i)].end-tokens[(*i)].start;
            // We may use strndup() to fetch string value
            memcpy(component, config + tokens[(*i)].start, size);
            component[size] = '\0';

            comp_idx = fnv1a_hash(component) % COMP_TABLE_SIZE;
            (*i)++;
        } 
        else if (jsoneq(config, &tokens[(*i)], ERPC_PARAMS_KEY) == 0)
        {
            (*i)++;  // processed key token
            if (tokens[(*i)].type != JSMN_ARRAY)
            {
                ESP_LOGE(TAG, "Params array expected");
                return EXIT_FAILURE;
            }
            int array_size = tokens[(*i)].size;
            (*i)++;  // processed array token
            for (params_id = 1; params_id <= array_size; params_id++)  // start at 1 ("name" is 1st params entry)
            {
                int size = tokens[(*i)].end - tokens[(*i)].start;
                memcpy(params[params_id], config + tokens[(*i)].start, size);
                params[params_id][size] = '\0';
                (*i)++;  // processed params value
            }
        }
        else 
        {
            ESP_LOGE(TAG, "Unexpected key: %.*s\n", tokens[(*i)].end-tokens[(*i)].start,
                config + tokens[(*i)].start);
        }
    }

    // Call the components init function and store the response as instance in inst_table
    inst_table[inst_idx] = comp_table[comp_idx](params_id, params);
    return EXIT_SUCCESS;
}


/// Call function on instance
///
/// Instance and function both are looked up using 'inst' and 'func' names
/// @param inst (const char*) - name of the component instance to use
/// @param func (const char*) - name of the function to call
/// @return EXIT_SUCCESS / EXIT_FAILURE
int frm_config_call(const char* inst, const char* func, int argc, frm_params_type argv)
{
    // lookup instance and function
    unsigned char inst_idx = fnv1a_hash((const unsigned char *)inst) % INST_TABLE_SIZE;
    unsigned char func_idx = fnv1a_hash((const unsigned char *)func) % FUNC_TABLE_SIZE;

    // call function with instance
    return func_table[func_idx](inst_table[inst_idx], argc, argv);
}

/// Get instance
///
/// Instance looked up using 'inst' name
/// @param inst (const char*) - name of the component instance to use
/// @return pointer to instance struct
void* frm_config_get_instance(const char* inst)
{
    unsigned char inst_idx = fnv1a_hash((const unsigned char *)inst) % INST_TABLE_SIZE;
    return inst_table[inst_idx];
}


/// Get function
///
/// Instance and function both are looked up using 'inst' and 'func' names
/// @param func (const char*) - name of the function to call
/// @return pointer to function
void* frm_config_get_function(const char* func)
{
    unsigned char func_idx = fnv1a_hash((const unsigned char *)func) % FUNC_TABLE_SIZE;
    return func_table[func_idx];
}
