# Hybrid 3D map snapshot contract

This contract is the sole public meaning of a Scout hybrid map. Protobuf v3 in
`proto/underwater/contracts/v1/mapping.proto` owns the wire fields. Mapping C++,
ROS 2, storage, and planner types are field-by-field adapters and MUST NOT add
public facts, fill absent safety values, or reinterpret enum values.

## Snapshot identity and time

`HybridMapSnapshot` is an immutable business object identified by the tuple
`map_id`, `map_version`, and `map_content_identity`. The version is strictly
increasing within one map ID. The content identity is the canonical SHA-256
defined in `HASHING.md`; version equality with different content identity is an integrity conflict.
A lower version is a version rollback.

| Map watermark input | Required outcome |
|---|---|
| exact map version + exact content identity | idempotent duplicate |
| same map version + different content identity | `INPUT_INVALID` |
| version rollback | `VERSION_INCOMPATIBLE` |
| higher version with invalid content | reject without advancing watermark |

`source_clock_domain_id` and `generated_at_monotonic_ns` belong to the mapping
producer's current boot session. `observed_at` is synchronized observation time
with explicit synchronization status and uncertainty. A consumer never compares
the producer's monotonic timestamp with its own monotonic clock. Freshness across
devices uses `observed_at` and the active versioned timing policy.

The map region and every grid use `mission_enu`, SI units, finite values, positive
resolutions, nonzero cell counts, and row-major X-fastest indexing:
`index = x + count_x * (y + count_y * z)`. A 2D grid omits the Z term. Grid origin
is the minimum-cell center. Array lengths MUST exactly equal the grid cell count;
neither truncation nor implicit padding is permitted.

The snapshot binds exact non-empty `ProfileRef` values for sensor extrinsics and
mapping parameters. Their ID, version, and 32-byte content identity are required.

## Required layers

A complete snapshot contains all of these independently meaningful layers:

- `SeafloorElevationLayer`: one elevation and quality value per 2D cell; quality
  is finite in `[0, 1]`.
- `VoxelOccupancyLayer`: one `VoxelState` per 3D cell.
- `EsdfLayer`: a declared signed or nonnegative convention and one finite SI
  distance per 3D cell.
- `AllowedWaterLayer`: one explicit allowed/not-allowed value per 3D cell.
- zero or more bounded `SemanticRegion` values with a stable class and ENU AABB.

All 3D grid geometries MUST be exactly identical. The 2D seafloor grid MUST share
their X/Y origin, resolution, and dimensions. A consumer does not resample at the
contract boundary.

Voxel interpretation is exhaustive and fail-closed:

| Input | Required consumer result |
|---|---|
| `FREE` | eligible for later safety evaluation; not execution authorization |
| `OCCUPIED` | obstacle |
| `UNKNOWN` | not traversable; may be a future survey target |
| `STALE` | not traversable |
| `CONFLICTED` | not traversable and diagnose inconsistent evidence |
| unspecified voxel state | reject the whole snapshot |
| unknown voxel state | reject the whole snapshot |

A seafloor elevation cell MUST NOT imply that the water column above it is free.
Suspended and overhanging obstacles are represented independently by occupied
3D voxels, even when the 2.5D seafloor below them is valid.

Unknown `EsdfConvention` or `SemanticClass` values reject the whole snapshot.
NaN, infinity, absent required scalar presence, inverted regions, invalid frame,
layer length mismatch, or inconsistent grid geometry also reject the whole
snapshot with `INPUT_INVALID`.

## Encoding and chunk reassembly

The uncompressed transfer payload is exactly the deterministic Protobuf
serialization of one `HybridMapSnapshot`. `payload_encoding` is
`MAP_PAYLOAD_DETERMINISTIC_PROTOBUF_V1`. The snapshot is serialized first, then
compressed as one byte stream according to the declared versioned `compression`,
then split into chunks. `MAP_COMPRESSION_NONE` is an identity transform.
`MAP_COMPRESSION_ZSTD_V1` means exactly one Zstandard frame with content size
present, frame checksum enabled, no dictionary, no concatenated/skippable frame,
and a declared window no larger than `maximum_map_snapshot_bytes`. Compression
level is not a wire fact. Peers that have not gated this rule in their exact
manifest MUST reject it.

Every chunk for one transfer has identical map identity, frame, chunk count,
encoding, compression, total sizes, full uncompressed SHA-256, and full
uncompressed IEEE CRC-32. `compressed_offset_bytes` is an offset into the single
compressed stream. `compressed_chunk_identity` is SHA-256 of the exact chunk
payload; `compressed_chunk_crc32` is IEEE CRC-32 of those same bytes.

A receiver performs these checks in order:

1. Reject unknown encoding/compression, invalid metadata, an over-limit total,
   or a chunk payload above the active `InterfaceLimits` before allocation.
2. Group only chunks with byte-identical transfer metadata. An identical repeat
   at the same index is idempotent; a conflicting repeat rejects the transfer.
3. Require exactly indexes `[0, chunk_count)` and contiguous declared offsets.
   Out-of-order delivery is allowed.
4. Verify each compressed chunk size, SHA-256, and CRC before concatenation.
5. Require the concatenated compressed size, decompress once, require the
   uncompressed total size, then verify the full uncompressed SHA-256 and CRC.
6. Parse one `HybridMapSnapshot`, reject unknown fields and trailing bytes, and
   apply all snapshot semantic checks before atomic publication.

No partial snapshot is observable to a planner. A receiver MUST NOT truncate,
partially parse, fill a missing chunk, or publish valid layers from an otherwise
invalid transfer. Hash and CRC are integrity/error-detection values, not source
authentication.

Both declared total sizes are independently bounded by
`maximum_map_snapshot_bytes`; chunk count, each compressed payload, cells per
layer, and semantic-region count are bounded by their respective active
`InterfaceLimits`. A violation is `RESOURCE_LIMIT_EXCEEDED`. The interface MUST
NOT truncate, downsample, skip a layer, or partially publish to satisfy a limit.

## Acknowledgement and retry

At the reassembly deadline, a receiver publishes `MAP_ACK_MISSING_CHUNKS` bound
to the exact `map_id`, `map_version`, full uncompressed content identity, and
expected chunk count. The missing indexes MUST be unique and strictly ascending
within `[0, expected_chunk_count)`. The sender retries only those exact chunks;
it does not silently start a transfer with different metadata under the old map
identity. No layer or partial snapshot is published while chunks are missing.

`acknowledgement_version` is strictly increasing for that exact transfer in the
receiver producer session. Same version and same `ack_content_identity` is an
idempotent duplicate; same version with different content is `INPUT_INVALID`;
rollback is `SEQUENCE_REJECTED`. `MAP_ACK_ACCEPTED` is emitted only after full
reassembly and semantic validation. Any CRC/hash error, conflicting duplicate,
unknown safe enum, malformed snapshot, or resource-limit violation emits
`MAP_ACK_REJECTED` with a stable `CodeRef`; it is never reported as missing data.
