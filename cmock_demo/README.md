# demo unity & cmock use 

build system is CMake


# cmock installation 

Ceedling contains a working CMock installation incl. all dependencies. I installed ceedling into my local home dir:

$ gem install --user-install ceedling

goes into: /home/mark/.gem/ruby/2.5.0/gems/ceedling-0.28.3

"/home/mark/.gem/ruby/2.5.0/gems/ceedling-0.28.3/vendor/cmock/" includes a working cmock installation with all required stuff like the missing "vendor/unity"


# use cmock.rb from the command line

$ mkdir mocks
$ ruby /home/mark/devel/10_esp32/esp32/cmock_demo/tools/cmock/lib/cmock.rb include/dep_demo.h


# cmock.rb mock_prefix option (mock_)

$ ruby /home/mark/devel/10_esp32/esp32/cmock_demo/components/cmock/lib/cmock.rb --mock_prefix=mock_ include/dep_demo.h


# resources

* [troubleshooting CMock on the command line](https://github.com/ThrowTheSwitch/CMock/blob/master/docs/CMock_Summary.md#mocking-from-the-command-line)
* [CMock options](https://github.com/ThrowTheSwitch/CMock/blob/master/docs/CMock_Summary.md#config-options)
* [ESP-IDF CMake build system](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system-cmake.html#)
* [this code got me started](https://github.com/mihaiolteanu/calculator)
