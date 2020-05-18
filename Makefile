.PHONY: build_docker
build_docker:
	sudo docker build -t zense_grpc_pywrapper .

.PHONY: run_docker
run_docker:
	xhost + local:root
	sudo docker run -it \
    --network="host" \
	--env=DISPLAY=$(DISPLAY) \
	--env=QT_X11_NO_MITSHM=1 \
	--privileged \
	--mount type=bind,src=/dev,dst=/dev,readonly \
	--mount type=bind,src=`pwd`,dst=/app \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	 zense_grpc_pywrapper

.PHONY: build
build:
	python setup.py build_ext --inplace

.PHONY: clean
clean:
	rm -rf build
	rm -rf *.c
	rm -rf *.so
