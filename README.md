# MTL-Meshroom

A heavily modified fork of [AliceVision](https://github.com/alicevision/AliceVision) with a working Apple Metal backend. This is mostly just a proof-of-concept.

## IMPORTANT INFORMATION

This project should compile on recent Macs, however, note that this build currently produces a bundle incompatibe with the upstream Meshroom project[^1].

There is a WIP effort to port the relevant changes to the upstream AliceVision repository, including important infrastructural changes to enable regular builds on macOS at all.

## Current Status

Upstreaming will be a multi-part process:

Currently, I opened a draft PR which should re-add the infrastructure needed to build AliceVision on macOS at all. Feel free to participate in the RFC, test out the build process and report any issues: [RFC: Re-add support for macOS](https://github.com/alicevision/AliceVision/pull/2019).

Once merged, I will attempt to port the DepthMap changes to the upstream project.

[^1]: You can make it work by adapting various hardcoded paths in Meshrooms `__init__.py` scripts.
