# frm_config component

# goal & features

goals:

* small
* fast
* reliable
* testable

... not really supprising goals for an embedded project!

features:

* configure (NOT program) the actual device from available sensors and actors implementations
* device functionality (sensors and actors) can be added later on without impacting existing devices
* configuration is sent/ updated via mqtt to a device so json would be a suitable data format
* (C programming language)


# acknowledgements

## jsmn

Jasmin is a great name for a json parser. I did not know [jsmn](https://github.com/zserge/jsmn) project before but when I researched available embedded json parsers in C it clearly stuck out as the best option. jsmn is already contained as a component in esp-idf.


## erpc

The frm_config component code is based on [erpc: abb6e5e](https://github.com/projectiota/erpc/commit/abb6e5e63ea728aab49372448295273c710717a9). erpc tremendously helped to quickstart the development of this component. Big thank you to [Drasko Draskovic](https://github.com/drasko) for developing and open sourcing the code.


# resources

* [Comparison and microbenchmark of C JSON parsers](https://lionet.livejournal.com/118853.html)
* [jsmn examples](https://alisdair.mcdiarmid.org/jsmn-example/)
