# vector-sim

The simulator of each Vector skill, as one npm package: `@vector/sim`, in `js/`. Browsers, the environment and
experts run the same JavaScript on the same MuJoCo `.wasm` (`@mujoco/mujoco`, single-thread), so equal inputs give
bit-equal states.

Scenes, assets and task limits come from vector-env; this package only runs them.

## Layout

| path | what |
|---|---|
| `js/` | the npm package `@vector/sim` (`package.json`, `package-lock.json`) |
| `js/skills/<skill>/` | one skill's engine: `src/`, `bin/`, `test/`, `tools/`, its `README.md` (guide and API) and `spec.md` |
| `js/test/` | tests of the package itself: every export and bin resolves |
| `scripts/setup.sh` | installs the package's dependencies (`npm ci` in `js/`) |
| [`spec.md`](spec.md) | the repository spec |

## Skills

| skill | engine |
|---|---|
| `bin-picking` | [js/skills/bin-picking/README.md](js/skills/bin-picking/README.md) |

## Imports

```js
import { Cell, Episode } from '@vector/sim';             // the bin-picking engine
import { mujoco, bundleFiles } from '@vector/sim/node.js'; // one file of the bin-picking engine
import { TryMonitor } from '@vector/sim/skills/bin-picking/src/rules.js';
```

- `@vector/sim` and `@vector/sim/<file>.js` are the bin-picking engine (`js/skills/bin-picking/src/`).
- A skill's files are also `@vector/sim/skills/<skill>/src/<file>.js`.
- `@vector/sim/testing` holds the test helpers; the bins are `vector-verify` and `vector-verify-unit`.

A dependent installs the package from its folder: `"@vector/sim": "file:<path to vector-sim>/js"`.

## Rounds

What miners run for a round is the git tag `<leaf-id>-r<n>` in vector-miner, vector-env and vector-sim. Round 1 of
`bin-picking-expert` is `bin-picking-expert-r1`.

An expert that plans with vector-sim builds against the tag of its round, so the same rows give the same states:

```bash
git clone --branch bin-picking-expert-r1 https://github.com/robotensor/vector-sim
```

## Setup and tests

Node 22 or newer (`.nvmrc`: 24).

```bash
git clone https://github.com/robotensor/vector-sim && cd vector-sim
scripts/setup.sh
cd js && npm test
```

`npm test` runs the package tests (`js/test/`) and the tests of every skill (`js/skills/<skill>/test/`). The skill
tests need that skill's asset bundle and find it as the skill's README says (bin-picking:
[Setup and tests](js/skills/bin-picking/README.md#setup-and-tests)). With no bundle they are skipped and say why.

## License

Apache-2.0 ([LICENSE](LICENSE)).
