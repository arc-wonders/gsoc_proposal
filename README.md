# GSoC 2026 Proposal: Improving Robustness of Kornia-SLAM for Real-World Visual-Inertial Deployment with Bubbaloop Integration

**Author:** Arkin Kansra ([@arc-wonders](https://github.com/arc-wonders))  
**Organization:** Kornia  
**Project Size:** Large (~350 hours)  
**Difficulty:** Hard

## Summary

This proposal focuses on making kornia-slam robust enough for continuous, unattended operation. The core work targets preprocessing (CLAHE, blur detection, adaptive FAST), tracking hardening (chi-square outlier rejection, adaptive guided matching), keyframe/map management (redundant culling, geometric insertion criteria), and failure recovery (LOST state, BoW-based relocalization). Stretch goals include a Schur complement BA prototype and adaptive runtime monitoring. A Bubbaloop application demonstrates the improved pipeline as an autonomous indoor mapping agent with MCP-based map serving.

## Contents

| File | Description |
|---|---|
| [GSoC_Proposal.md](GSoC_Proposal.md) | Full proposal (12 sections) |

## Quick Links

- [kornia-rs](https://github.com/kornia/kornia-rs) · [Bubbaloop](https://github.com/kornia/bubbaloop) · [kornia-slam](https://github.com/kornia/kornia-slam) · [kornia](https://github.com/kornia/kornia)
