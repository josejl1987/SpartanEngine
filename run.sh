#!/bin/bash
cd /home/jose/SpartanEngine

# Keep runtime shader directory in sync for launches from binaries/
rsync -a --delete data/shaders/ binaries/data/shaders/

LD_LIBRARY_PATH=third_party/install/lib:$LD_LIBRARY_PATH binaries/spartan_vulkan_debug "$@"
