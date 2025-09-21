#!/usr/bin/env bash
git lfs install --system
git lfs pull --include="urdf/**"  
git submodule update --init
