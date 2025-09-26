# MTL-Meshroom

A heavily modified fork of [AliceVision](https://github.com/alicevision/AliceVision) with a working Apple Metal backend. This is mostly just a proof-of-concept.

## IMPORTANT INFORMATION

This project should compile on recent Macs, however, note that this build currently produces a bundle incompatibe with the upstream Meshroom project[^1].

There is a WIP effort to port the relevant changes to the upstream AliceVision repository, including important infrastructural changes to enable regular builds on macOS at all.

[^1]: You can make it work by adapting various hardcoded paths in Meshrooms `__init__.py` scripts.
