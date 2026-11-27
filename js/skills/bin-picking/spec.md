# vector-sim: bin-picking engine spec

The bin-picking cell engine: a UR10e arm with a Robotiq 2F-85 gripper over a bin of parts, simulated by
`@mujoco/mujoco` (single-thread wasm). Browsers, the environment and experts run this code. It is shared by every
bin-picking leaf.

## 1. Files

| path | content |
|---|---|
| `src/actions.js`, `src/cell.js`, `src/episode.js`, `src/hash.js`, `src/obs.js`, `src/picks.js`, `src/plan.js`, `src/rules.js` | the pinned files (section 2); `picks.js` runs several picks per pile scene (`PickRun`, `replayPicks`, `pickCount`) |
| `src/ik.js`, `src/driver.js`, `src/replay.js` | IK, closed-loop try, replay |
| `src/node.js` | Node only: `mujoco()`, `bundleFiles(dir, scene)`, `inside(root, p)`, `PIN_FILES`, `simPin()` |
| `src/bundle.js` | Node only: `MISSING`, `envDir()`, `pinnedVersion(dir)`, `findBundle()` (section 3) |
| `src/index.js` | the exports of `@vector/sim` |
| `bin/verify.mjs`, `bin/verify_unit.mjs` | replay an episode log / one unit from stdin, print the result |
| `test/*.test.js`, `test/helpers.js`, `test/fixtures/` | tests; `helpers.js` is `@vector/sim/testing` |
| `tools/dump_states.mjs` | per-tick state dump of a scene |
| `README.md` | the engine's API |

All of these moved together from `js/src/`, `js/bin/`, `js/test/` and `js/tools/`, so their relative imports are
unchanged.

## 2. Pinned files and `simPin()`

- `PIN_FILES` is `['actions.js', 'cell.js', 'episode.js', 'hash.js', 'obs.js', 'picks.js', 'plan.js', 'rules.js']`:
  every file whose code decides states, targets or verdicts.
- `simPin()` returns `{ sim, code, mujoco, wasm }`:
  - `sim`: `version` of the package's `package.json` (`js/package.json`, three folders above `src/`);
  - `code`: sha256 over `name\0content\0` of each pinned file, in `PIN_FILES` order, names without folders;
  - `mujoco`: the version of `@mujoco/mujoco`; `wasm`: sha256 of the `mujoco.wasm` it loads.
- The four values equal those of the previous layout. A runtime whose `code` or `wasm` differs is another engine.

## 3. Finding the asset bundle

`findBundle()` returns the first bundle folder (a folder with `manifest.json`) of:

1. `VECTOR_ENV_BUNDLE`, else `VECTOR_SIM_BUNDLE` (used as given: `null` when it is not a bundle folder);
2. `<vector-env>/bundle`;
3. `<cache>/<version>`: `<cache>` is `VECTOR_ENV_CACHE`, default `~/.cache/vector-env/assets`.

- `<vector-env>` is `envDir()`: `VECTOR_ENV_DIR`, default `vector-env` beside this checkout (five folders above
  `src/`, then `vector-env`).
- `<version>` is `pinnedVersion()`: `VECTOR_ENV_ASSETS_VERSION`, else `assets.version` read from
  `<vector-env>/vector_env/skills/bin_picking/env.yaml`, else from `<vector-env>/vector_env/env.yaml`, else `null`.
- `MISSING` is the one-line message used when there is no bundle.

`bundleFiles(dir, scene)` reads the meshes a scene names and refuses any path outside the bundle folder (absolute,
`.`/`..`/empty segments, backslash, NUL).

## 4. Test helpers (`@vector/sim/testing`)

`test/helpers.js` exports `ROOT` (this checkout's root folder), `ASSETS` (`findBundle()`), `SKIP` (`false`, or
`MISSING` when there is no bundle), `SETTINGS`, `BUNDLE` (the fixture bundle built from `ASSETS`), `compose`,
`recordedPick` (rows recorded on the fixture scenes, read from `test/fixtures/picks.json`) and `highestPart`.
Names and meanings are unchanged.

## 5. Formats

Unchanged: `vector-episode/2` (`LOG_FORMAT`), `vector-actions/1` (`ACTIONS_FORMAT`), `vector-console/1`
(`CHAIN_FORMAT`), `EMPTY_ROWS_HASH` `cbf29ce484222325`, rows of 7 values (`CTRL`).

## 6. Tests

`npm test` in `js/` runs `test/*.test.js` here on the fixture scenes; without a bundle they skip and say why.
