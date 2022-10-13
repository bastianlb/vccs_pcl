

```
docker build . -t vccs
docker run --user $(id -u):$(id -g) -v $PWD/:/vccs --rm -it vccs:latest

```

for visualization run
```
xhost +local:root
docker run --user $(id -u):$(id -g) -v $PWD/:/vccs --rm -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" vccs:latest

```
