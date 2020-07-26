.PHONY: plugin.dbg.so
plugin.dbg.so:
	cd Core/ && \
	mkdir -p build/ && \
	cd build/ && \
	cmake ../src && \
	make "-j$(nproc)" && \
	cd ../.. && \
	mkdir -p plugin/build/ && \
	cd plugin/build/ && \
	cmake .. && \
	make "-j$(nproc)" && \
	cd ../.. && \
	rm -f plugin.dbg.so && \
	ln -s plugin/build/libplugin.so plugin.dbg.so && \
	true

.PHONY: plugin.opt.so
plugin.opt.so:
	mkdir -p plugin/build/ && \
	cd plugin/build/ && \
	cmake .. -DDNDEBUG=1 -DCMAKE_BUILD_TYPE=Release .. && \
	make "-j$(nproc)" && \
	cd ../../.. && \
	rm -f plugin.opt.so && \
	ln -s plugin/build/libplugin.so plugin.opt.so && \
	true

.PHONY: clean
clean:
	touch plugin/build && rm -rf plugin/build *.so && touch Core/build && rm -rf Core/build *.so
