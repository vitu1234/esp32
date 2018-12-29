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

#ifndef FRM_CONFIG_H
#define FRM_CONFIG_H

//#define FUNC_TABLE_SIZE 1024
//#define COMP_TABLE_SIZE 1024
//#define INST_TABLE_SIZE 1024
//typedef unsigned char JSMN_PARAMS_t[8][16];
typedef unsigned char frm_params_type[8][16];

void frm_config_add_component(char* cmp_name, void (*f)(int argc, frm_params_type argv));
void frm_config_add_function(char* func_name, void (*f)(void * inst));
int frm_config_component_init(const char* req);
int frm_config_call(const char* inst, const char* func);

#endif /** FRM_CONFIG_H */
