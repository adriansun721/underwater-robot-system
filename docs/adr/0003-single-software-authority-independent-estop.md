# One software ExecutionAuthority per physical domain with independent hard E-stop paths

Each physical execution authority domain has exactly one logical
ExecutionAuthority. The main laying domain owns track, payout, and tension
grants on the main NUC; the Scout motion domain owns FCU-motion grants on the
Scout NUC. An authority MUST NOT sign, renew, revoke, or share watermarks across domains,
and a grant received across devices must be revalidated and re-issued in the
target domain's local monotonic clock. Each domain retains an independent hard
E-stop path so immediate local action never depends on either software
authority; normalized E-stop state remains single-owner within its own state
domain.
