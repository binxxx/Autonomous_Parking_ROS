#! /bin/bash
echo "@Generating the map..."
./map.generator 38 33 0.1 100 160 180 10
echo "@Copying map into car_description/map folder..."
cp map/map.yaml ../car_description/map
echo "@Done."
