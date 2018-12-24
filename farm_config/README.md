# Farm components

# project layout

we follow the normal ESP-IDF layout

```
unit_test                      — Application project directory
  - components                 — Components of the application project
    + farm_config
  + main                       - Main source files of the application project
  + test                       — Test project directory
  Makefile / CMakeLists.txt    - Makefiles of the application project
```



# Running tests

bla

running the tests:

* Enter `test` directory (`unit_test/test`), and run `idf.py -p PORT flash monitor` if you are using CMake build system.
* follow the instructions in the menu or use "*[Enter]" to run all tests
* Observe the output: results of test case execution.
* use "[Ctrl]-]" to leave the monitor


