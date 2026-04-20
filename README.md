# MTL-Meshroom

A heavily modified fork of [AliceVision](https://github.com/alicevision/AliceVision) with a working Apple Metal backend. This is mostly just a proof-of-concept.

## IMPORTANT INFORMATION

This project should compile on recent Macs, however, note that this build currently produces a bundle incompatibe with the upstream Meshroom project[^1].

There is a WIP effort to port the relevant changes to the upstream AliceVision repository, including important infrastructural changes to enable regular builds on macOS at all.

## Current Status

The PR carrying important infrastructure changes for re-enabling macOS support has been merged: [Re-add support for macOS](https://github.com/alicevision/AliceVision/pull/2019).

I initially planned on upstreaming the Metal DepthMap backend as well. However, that might actually be obsolete because there is a unified backend using SYCL in the works. I will test out if AdaptiveCpp's Metal backend is stable and good enough to support the DepthMap library and if so, consider dropping the native Metal backend in favor of the former.

[^1]: You can make it work by adapting various hardcoded paths in Meshrooms `__init__.py` scripts.
