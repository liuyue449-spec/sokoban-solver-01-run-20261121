# Sokoban solver — make.fun `sokoban-solver-01-run-20261121`

A push-optimal Sokoban solver written from scratch for the make.fun bounty
"Build the best Sokoban solver, 21 November 2026" ($100 in USDC).

## How it works

Every level in this competition is a one-cell-wide hallway with one to four
rooms opening off it (one door each). A box pushed out of a room through its
door can never come back, so each room is an independent problem and the
level's push-optimal count is the sum of the rooms' optima. The solver:

1. splits the level into hallway + rooms;
2. solves each room push-optimally with A* over pushes — an admissible
   minimum-assignment heuristic (box-to-goal walking distances) plus
   deadlock pruning (dead squares, wall runs, 2x2 freezes);
3. stitches the rooms together with shortest player walks through the
   hallway, trying room orders until the walks between rooms succeed;
4. replays the whole move string on the level and only writes it out when
   the replay is legal and solves the level.

No clock, no randomness, no network: search is cut off by a fixed number of
node expansions, so two runs produce byte-identical outputs.

## Run

```
node main.ts [inputDir] [outputDir]
```

Defaults are `/input` and `/output` (or `MAKEFUN_INPUT_DIR` /
`MAKEFUN_OUTPUT_DIR` when the harness sets them). Reads
`<inputDir>/i001..i005/level.txt`, writes
`<outputDir>/i001..i005/solution.txt` (player moves in `u`/`d`/`l`/`r`).

## Results (practice set)

| instance | pushes | push-optimal | points |
|----------|--------|--------------|--------|
| i001 | 13 | 13 | 1000 |
| i002 | 16 | 16 | 1000 |
| i003 | 29 | 29 | 1000 |
| i004 | 47 | 47 | 1000 |
| i005 | 60 | 60 | 1000 |

Score **1,000,000 / 1,000,000** on the practice set, verified with the kit's
own `grade-local` (two executions, byte-identical). Eight further freshly
drawn sets from the kit's generator also all scored 1,000,000, each solved
in about a second (limit: 60 s for the whole set).
