# Protobuf for NUC semantics and a fixed CAN contract for STM32F4

NUC-level and dual-robot messages use Protobuf v3 as their sole semantic
schema, while the constrained STM32F4 link uses a DBC/fixed CAN protocol.
ROS and C++ types remain adapters. The split accepts two wire authorities but
keeps units, codes, ranges, versions, and compatibility joined by one registry,
ContractManifest, and conformance suite instead of forcing an unsuitable
memory layout across both boundaries.

