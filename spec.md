# vector-sim spec

vector-sim holds the simulator of each Vector skill as one npm package, `@vector/sim`. This file covers the
repository; each skill's engine has its own spec next to its code (`js/skills/<skill>/spec.md`).

## 1. Layout

| path | what |
|---|---|
| `js/` | the npm package `@vector/sim`: package root, `package.json`, `package-lock.json`, `node_modules/` (ignored) |
| `js/skills/<skill>/` | one skill's engine: `src/`, `bin/`, `test/`, `tools/`, `README.md`, `spec.md` |
| `js/test/` | tests of the package itself: every export and bin resolves |
| `scripts/setup.sh` | `npm ci` in `js/` |
| `README.md`, `spec.md`, `LICENSE`, `.nvmrc`, `.gitignore` | the repository |

Skills: `bin-picking` (`js/skills/bin-picking/`).

There is no `js/src/`, `js/bin/`, `js/tools/` and no test file directly in `js/skills/`. Shared files name no
skill, except the compatibility entries of `js/package.json` (section 2).

## 2. Package

`js/package.json` keeps `name` (`@vector/sim`), `version`, `type` (`module`), `engines`, `dependencies` and
`license`. It sets:

- `exports`:

  | specifier | file |
  |---|---|
  | `.` | `./skills/bin-picking/src/index.js` |
  | `./testing` | `./skills/bin-picking/test/helpers.js` |
  | `./skills/*` | `./skills/*` |
  | `./*` | `./skills/bin-picking/src/*` |

- `bin`: `vector-verify` → `./skills/bin-picking/bin/verify.mjs`, `vector-verify-unit` →
  `./skills/bin-picking/bin/verify_unit.mjs`.
- `scripts.test`: `node --test test/*.test.js skills/*/test/*.test.js`.

`package-lock.json` is regenerated from `package.json` with the same dependency versions.

Every specifier that resolved before resolves to a file with the same code: `@vector/sim`, `@vector/sim/testing`
and `@vector/sim/<file>.js` for each file of `js/skills/bin-picking/src/`. A skill's files can also be imported as
`@vector/sim/skills/<skill>/src/<file>.js`.

## 3. Dependents

- The package root stays `js/`. Dependents keep `"@vector/sim": "file:<path to vector-sim>/js"`, and images keep
  fetching the repository and running `npm ci --omit=dev` in `js/`.
- A dependent's `npm ci` needs nothing else from this repository.

## 4. Rounds

What miners run for a round is the git tag `<leaf-id>-r<n>` in vector-miner, vector-env and vector-sim (round 1 of
`bin-picking-expert` is `bin-picking-expert-r1`). Pins name these tags. No `v<number>` tag is used as a pin.

## 5. Behaviour

Moving code between folders changes no result: the same scene and rows give bit-equal states, hashes, verdicts
and `simPin()` before and after. Each skill's spec lists what must stay byte-identical.

## 6. Tests

```bash
scripts/setup.sh
cd js && npm test
```

`js/test/exports.test.js` checks section 2: each specifier and bin resolves to an existing file, and
`@vector/sim/<file>.js` is the same module as `@vector/sim/skills/bin-picking/src/<file>.js`.
