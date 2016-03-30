# stunning-pancake

#beagle bone installation 
pip install --upgrade PyBBIO

# test:
cd /usr/local/lib/PyBBIO/examples/
python blinky.py

# modify PyBBIO dts to works with bb green
sed -i 's/\"ti,beaglebone-black\"/\"ti,beaglebone-black\", \"ti,beaglebone-green\"/g' tools/overlays/*.dts
