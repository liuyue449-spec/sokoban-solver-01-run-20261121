// Sokoban solver for make.fun "sokoban-solver-01-run-20261121".
//
// Written from scratch in this session. No kit code, no outside solver code.
//
// Idea: every level in this competition is a one-cell-wide hallway with one to
// four rooms opening off it (one door each). A box pushed out of a room through
// its door can never come back, so each room is independent and the level's
// push-optimal count is the sum of the rooms' optima. The solver therefore
//   1. splits the level into hallway + rooms,
//   2. solves each room push-optimally with A* over pushes (an admissible
//      min-assignment heuristic over box-to-goal walking distances, plus
//      deadlock pruning: dead squares, wall runs, 2x2 freezes),
//   3. stitches the rooms together with shortest player walks through the
//      hallway (trying room orders until the walk between rooms succeeds),
//   4. replays the whole move string on the level and only writes it out
//      when the replay is legal and solves the level.
//
// Determinism: no clock, no randomness, no environment reads. Search is cut
// off by a fixed number of node expansions, never by elapsed time, so two
// runs produce byte-identical outputs.
//
// Usage: node main.ts [inputDir] [outputDir]
//   defaults: /input and /output. Reads <inputDir>/i001..i005/level.txt and
//   writes <outputDir>/i001..i005/solution.txt (LURD moves, u/d/l/r).

import * as fs from "node:fs";
import * as path from "node:path";

// ---------------------------------------------------------------------------
// Directions. Fixed order everywhere, so every search is deterministic.
// ---------------------------------------------------------------------------
const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
  { dx: 1, dy: 0, ch: "r" },
];

// ---------------------------------------------------------------------------
// Level parsing (XSB: # wall, space floor, . goal, $ box, * box on goal,
// @ player, + player on goal).
// ---------------------------------------------------------------------------
interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall (or outside the ragged rows)
  goals: Set<number>;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.replace(/\r/g, "").split("\n");
  while (rows.length > 0 && rows[rows.length - 1] === "") rows.pop();
  const h = rows.length;
  const w = rows.reduce((m, r) => Math.max(m, r.length), 0);
  const wall = new Uint8Array(w * h).fill(1);
  const goals = new Set<number>();
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < rows[y].length; x++) {
      const c = rows[y][x];
      const i = y * w + x;
      if (c !== "#") wall[i] = 0;
      if (c === "." || c === "*" || c === "+") goals.add(i);
      if (c === "$" || c === "*") boxes.push(i);
      if (c === "@" || c === "+") player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// ---------------------------------------------------------------------------
// Decomposition into hallway + rooms.
// ---------------------------------------------------------------------------
interface Room {
  cells: number[]; // room cells (global indices), sorted
  goals: number[]; // goal cells inside the room
  boxes: number[]; // starting box cells inside the room
  entry: number; // hallway cell the player stands on to enter
  door: number; // room cell adjacent to the entry cell
}

interface Decomposition {
  hallway: Set<number>;
  rooms: Room[];
}

function connectedComponents(open: Uint8Array, w: number, h: number): number[][] {
  const seen = new Uint8Array(w * h);
  const comps: number[][] = [];
  for (let s = 0; s < w * h; s++) {
    if (!open[s] || seen[s]) continue;
    const comp: number[] = [];
    const stack = [s];
    seen[s] = 1;
    while (stack.length > 0) {
      const c = stack.pop() as number;
      comp.push(c);
      const x = c % w;
      const y = (c / w) | 0;
      for (const d of DIRS) {
        const nx = x + d.dx;
        const ny = y + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (open[ni] && !seen[ni]) {
          seen[ni] = 1;
          stack.push(ni);
        }
      }
    }
    comps.push(comp);
  }
  return comps;
}

// Try one hallway candidate (a list of global cell indices). Returns rooms or
// null when the split does not account for every box and goal.
function tryHallway(level: Level, hallCells: number[]): Decomposition | null {
  const { w, h, wall } = level;
  const hall = new Set<number>(hallCells);
  const open = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) open[i] = wall[i] === 0 && !hall.has(i) ? 1 : 0;
  const comps = connectedComponents(open, w, h);
  const rooms: Room[] = [];
  let accountedBoxes = 0;
  let accountedGoals = 0;
  for (const comp of comps) {
    const compSet = new Set<number>(comp);
    const boxes = level.boxes.filter((b) => compSet.has(b));
    const goals = [...level.goals].filter((g) => compSet.has(g));
    if (boxes.length === 0 && goals.length === 0) continue; // empty alcove
    if (boxes.length === 0 || boxes.length !== goals.length) return null;
    // The door: the (single) room cell touching the hallway.
    let door = -1;
    let entry = -1;
    for (const c of comp) {
      const x = c % w;
      const y = (c / w) | 0;
      for (const d of DIRS) {
        const nx = x + d.dx;
        const ny = y + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (hall.has(ni)) {
          if (door === -1 || c < door || (c === door && ni < entry)) {
            door = c;
            entry = ni;
          }
        }
      }
    }
    if (door === -1) return null;
    // Exactly one door cell: no second room cell may touch the hallway.
    for (const c of comp) {
      if (c === door) continue;
      const x = c % w;
      const y = (c / w) | 0;
      for (const d of DIRS) {
        const nx = x + d.dx;
        const ny = y + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (hall.has(ny * w + nx)) return null;
      }
    }
    comp.sort((a, b) => a - b);
    goals.sort((a, b) => a - b);
    boxes.sort((a, b) => a - b);
    rooms.push({ cells: comp, goals, boxes, entry, door });
    accountedBoxes += boxes.length;
    accountedGoals += goals.length;
  }
  if (accountedBoxes !== level.boxes.length) return null;
  if (accountedGoals !== level.goals.size) return null;
  if (rooms.length === 0) return null;
  return { hallway: hall, rooms };
}

function decompose(level: Level): Decomposition | null {
  const { w, h, wall } = level;
  const px = level.player % w;
  const py = (level.player / w) | 0;
  // Candidate 1: the player's row, when it is one contiguous open run.
  const rowCells: number[] = [];
  for (let x = 0; x < w; x++) if (wall[py * w + x] === 0) rowCells.push(py * w + x);
  if (
    rowCells.length >= 4 &&
    rowCells.includes(level.player) &&
    rowCells.every((c, i) => i === 0 || c === rowCells[i - 1] + 1)
  ) {
    const d = tryHallway(level, rowCells);
    if (d) return d;
  }
  // Candidate 2: the player's column, same test.
  const colCells: number[] = [];
  for (let y = 0; y < h; y++) if (wall[y * w + px] === 0) colCells.push(y * w + px);
  if (
    colCells.length >= 4 &&
    colCells.includes(level.player) &&
    colCells.every((c, i) => i === 0 || c === colCells[i - 1] + w)
  ) {
    const d = tryHallway(level, colCells);
    if (d) return d;
  }
  return null;
}

// ---------------------------------------------------------------------------
// Shortest player walk (BFS) over open cells, treating box cells as blocked.
// Returns the move string, or null when unreachable. Fixed neighbour order.
// ---------------------------------------------------------------------------
function walkPath(
  level: Level,
  from: number,
  to: number,
  boxSet: Set<number>,
): string | null {
  if (from === to) return "";
  const { w, h, wall } = level;
  const prev = new Int32Array(w * h).fill(-2);
  const prevCh = new Array<string>(w * h).fill("");
  prev[from] = -1;
  const queue = [from];
  let qi = 0;
  while (qi < queue.length) {
    const c = queue[qi++];
    const x = c % w;
    const y = (c / w) | 0;
    for (const d of DIRS) {
      const nx = x + d.dx;
      const ny = y + d.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const ni = ny * w + nx;
      if (wall[ni] !== 0 || boxSet.has(ni) || prev[ni] !== -2) continue;
      prev[ni] = c;
      prevCh[ni] = d.ch;
      if (ni === to) {
        let s = "";
        let cur = ni;
        while (cur !== from) {
          s = prevCh[cur] + s;
          cur = prev[cur];
        }
        return s;
      }
      queue.push(ni);
    }
  }
  return null;
}

// ---------------------------------------------------------------------------
// Push-optimal A* for one room.
//
// Local indexing: room cells get local ids 0..n-1, the entry cell is local n.
// Boxes live on room cells only; the player may stand on room cells or the
// entry cell. A state is (sorted box cells, player reachable region). The
// region is canonicalised to its smallest local cell, so the state key is
// one integer: packed boxes * 64 + region minimum.
// ---------------------------------------------------------------------------
interface RoomSolution {
  moves: string; // player moves solving the room, starting at the entry cell
  endPlayer: number; // global cell where the player ends up
  endBoxes: number[]; // global cells of the boxes at the end (all on goals)
  pushes: number;
  expansions: number;
}

function permutations(n: number): number[][] {
  const out: number[][] = [];
  const a = Array.from({ length: n }, (_, i) => i);
  const gen = (k: number) => {
    if (k === 1) {
      out.push([...a]);
      return;
    }
    for (let i = 0; i < k; i++) {
      gen(k - 1);
      const j = k % 2 === 0 ? i : 0;
      [a[j], a[k - 1]] = [a[k - 1], a[j]];
    }
  };
  gen(n);
  return out;
}

class RoomSolver {
  private level: Level;
  private room: Room;
  private n: number; // room cells
  private N: number; // room cells + entry cell
  private localOf: Map<number, number>;
  private globalOf: number[];
  private isRoomCell: Uint8Array; // over local ids 0..N-1 (entry counts as not-room)
  private goalLocal: Set<number>;
  private goalsLocal: number[];
  private startBoxesLocal: number[];
  private dist: Int16Array; // dist[a * N + b], box walking distances over room cells
  private dead: Uint8Array; // dead-square cells (room cells only)
  private perms: number[][];
  private totalExpansions = 0;

  constructor(level: Level, room: Room) {
    this.level = level;
    this.room = room;
    this.n = room.cells.length;
    this.N = this.n + 1;
    this.localOf = new Map<number, number>();
    this.globalOf = [...room.cells, room.entry];
    room.cells.forEach((g, i) => this.localOf.set(g, i));
    this.localOf.set(room.entry, this.n);
    this.isRoomCell = new Uint8Array(this.N);
    for (let i = 0; i < this.n; i++) this.isRoomCell[i] = 1;
    this.goalLocal = new Set<number>(room.goals.map((g) => this.localOf.get(g) as number));
    this.goalsLocal = room.goals.map((g) => this.localOf.get(g) as number);
    this.startBoxesLocal = room.boxes
      .map((g) => this.localOf.get(g) as number)
      .sort((a, b) => a - b);
    this.dist = this.computeDistances();
    this.dead = this.computeDeadCells();
    this.perms = permutations(room.boxes.length);
  }

  // BFS distances between room cells, ignoring boxes. -1 = unreachable.
  private computeDistances(): Int16Array {
    const N = this.N;
    const dist = new Int16Array(N * N).fill(-1);
    for (let s = 0; s < this.n; s++) {
      const q = [s];
      dist[s * N + s] = 0;
      let qi = 0;
      while (qi < q.length) {
        const c = q[qi++];
        const g = this.globalOf[c];
        const x = g % this.level.w;
        const y = (g / this.level.w) | 0;
        for (const d of DIRS) {
          const nx = x + d.dx;
          const ny = y + d.dy;
          if (nx < 0 || ny < 0 || nx >= this.level.w || ny >= this.level.h) continue;
          const ni = this.localOf.get(ny * this.level.w + nx);
          if (ni === undefined || ni >= this.n) continue;
          if (dist[s * N + ni] === -1) {
            dist[s * N + ni] = dist[s * N + c] + 1;
            q.push(ni);
          }
        }
      }
    }
    return dist;
  }

  // Static dead squares: corners and wall runs with no goal in them.
  //
  // Two notions of "blocked" matter here and they differ on the entry cell:
  // a box can never stand on the entry cell, but the player CAN stand there
  // and push a box up into the room. So "blocked" below means a real wall
  // (or outside the level): a side the player can never stand on. A wall
  // run only counts as dead when both of its ends are shut too: a box
  // slides along the run, and if either end opens onto a room cell the box
  // can slide out there and escape.
  private computeDeadCells(): Uint8Array {
    const dead = new Uint8Array(this.N);
    const { w, h } = this.level;
    // Real wall (or outside the level) at the neighbour in (dx, dy)?
    const walled = (li: number, dx: number, dy: number): boolean => {
      const g = this.globalOf[li];
      const x = g % w;
      const y = (g / w) | 0;
      const nx = x + dx;
      const ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) return true;
      return this.level.wall[ny * w + nx] === 1;
    };
    // Can a box slide into this global cell? No when it is a real wall,
    // outside the level, or the entry cell (boxes never leave the room).
    const slideBlocked = (g: number): boolean => {
      const x = g % w;
      const y = (g / w) | 0;
      if (x < 0 || y < 0 || x >= w || y >= h) return true;
      if (g === this.room.entry) return true;
      return this.level.wall[g] === 1;
    };
    for (let li = 0; li < this.n; li++) {
      if (this.goalLocal.has(li)) continue;
      const up = walled(li, 0, -1);
      const down = walled(li, 0, 1);
      const left = walled(li, -1, 0);
      const right = walled(li, 1, 0);
      if ((up || down) && (left || right)) dead[li] = 1; // corner
    }
    // Wall runs: for each side, group cells walled on that side into runs
    // along the wall; a run with no goal is dead only when both ends shut.
    for (const [dx, dy, px, py] of [
      [0, -1, 1, 0],
      [0, 1, 1, 0],
      [-1, 0, 0, 1],
      [1, 0, 0, 1],
    ] as const) {
      const seen = new Uint8Array(this.N);
      for (let li = 0; li < this.n; li++) {
        if (seen[li] || this.goalLocal.has(li) || !walled(li, dx, dy)) continue;
        // collect the run along (px, py)
        const run: number[] = [];
        const stack = [li];
        seen[li] = 1;
        while (stack.length > 0) {
          const c = stack.pop() as number;
          run.push(c);
          const g = this.globalOf[c];
          const x = g % w;
          const y = (g / w) | 0;
          for (const s of [-1, 1]) {
            const nx = x + px * s;
            const ny = y + py * s;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const ni = this.localOf.get(ny * w + nx);
            if (ni === undefined || ni >= this.n || seen[ni]) continue;
            if (!walled(ni, dx, dy)) continue;
            seen[ni] = 1;
            stack.push(ni);
          }
        }
        if (run.some((c) => this.goalLocal.has(c))) continue;
        // ends shut? Walk to each end of the run and look past it.
        let shut = true;
        for (const s of [-1, 1]) {
          let end = run[0];
          for (const c of run) {
            const cg = this.globalOf[c];
            const eg = this.globalOf[end];
            const along = (cg % w) * px + ((cg / w) | 0) * py;
            const eAlong = (eg % w) * px + ((eg / w) | 0) * py;
            if (s * along > s * eAlong) end = c;
          }
          const eg = this.globalOf[end];
          const ex = (eg % w) + px * s;
          const ey = ((eg / w) | 0) + py * s;
          if (ex < 0 || ey < 0 || ex >= w || ey >= h || !slideBlocked(ey * w + ex)) {
            shut = false;
          }
        }
        if (shut) for (const c of run) dead[c] = 1;
      }
    }
    return dead;
  }

  // Min-assignment heuristic: the cheapest way to match boxes to goals over
  // walking distances. Admissible: one push moves one box one cell.
  private heuristic(boxes: number[]): number {
    let best = Infinity;
    for (const perm of this.perms) {
      let s = 0;
      for (let i = 0; i < boxes.length; i++) {
        const d = this.dist[boxes[i] * this.N + this.goalsLocal[perm[i]]];
        if (d < 0) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  }

  // 2x2 freeze: a 2x2 block of walls/boxes holding a box that is not on a
  // goal can never come apart.
  private frozen(boxes: number[], boxSet: Set<number>): boolean {
    for (const b of boxes) {
      if (this.goalLocal.has(b)) continue;
      const g = this.globalOf[b];
      const x = g % this.level.w;
      const y = (g / this.level.w) | 0;
      for (const [ox, oy] of [
        [0, 0],
        [-1, 0],
        [0, -1],
        [-1, -1],
      ] as const) {
        let allBlocked = true;
        let anyBoxOffGoal = false;
        for (let k = 0; k < 4; k++) {
          const cx = x + ox + (k % 2);
          const cy = y + oy + ((k / 2) | 0);
          if (cx < 0 || cy < 0 || cx >= this.level.w || cy >= this.level.h) continue;
          const ni = this.localOf.get(cy * this.level.w + cx);
          if (ni === undefined || ni >= this.n) continue; // wall / outside room
          if (!boxSet.has(ni)) {
            allBlocked = false;
            break;
          }
          if (!this.goalLocal.has(ni)) anyBoxOffGoal = true;
        }
        if (allBlocked && anyBoxOffGoal) return true;
      }
    }
    return false;
  }

  private pack(boxes: number[]): number {
    let p = 0;
    for (const b of boxes) p = p * 64 + b;
    return p;
  }

  private unpack(packed: number, count: number): number[] {
    const boxes = new Array<number>(count);
    let p = packed;
    for (let i = count - 1; i >= 0; i--) {
      boxes[i] = p % 64;
      p = (p / 64) | 0;
    }
    return boxes;
  }

  // Flood fill from the player over room cells + entry, boxes as obstacles.
  // Returns the reachable set as a bitmask array plus its smallest cell.
  private reachable(playerLocal: number, boxSet: Set<number>): { set: Uint8Array; min: number } {
    const set = new Uint8Array(this.N);
    const q = [playerLocal];
    set[playerLocal] = 1;
    let min = playerLocal;
    let qi = 0;
    while (qi < q.length) {
      const c = q[qi++];
      const g = this.globalOf[c];
      const x = g % this.level.w;
      const y = (g / this.level.w) | 0;
      for (const d of DIRS) {
        const nx = x + d.dx;
        const ny = y + d.dy;
        if (nx < 0 || ny < 0 || nx >= this.level.w || ny >= this.level.h) continue;
        const ni = this.localOf.get(ny * this.level.w + nx);
        if (ni === undefined || set[ni] || boxSet.has(ni)) continue;
        set[ni] = 1;
        if (ni < min) min = ni;
        q.push(ni);
      }
    }
    return { set, min };
  }

  // A* over pushes. f = gw * g + hw * h. Returns the goal node index, or -1
  // when the expansion budget runs out (or the room is unsolvable).
  private search(gw: number, hw: number, maxExpansions: number): {
    goalNode: number;
    nodes: {
      parent: number[];
      g: number[];
      h: number[];
      player: number[];
      packed: number[];
      pushBox: number[];
      pushDir: number[];
    };
    expansions: number;
  } | null {
    const nb = this.startBoxesLocal.length;
    const parent: number[] = [-1];
    const g: number[] = [0];
    const h: number[] = [this.heuristic(this.startBoxesLocal)];
    const player: number[] = [this.n]; // start on the entry cell
    const packed: number[] = [this.pack(this.startBoxesLocal)];
    const pushBox: number[] = [-1];
    const pushDir: number[] = [-1];
    if (!isFinite(h[0])) return null;

    // Binary heap of node ids, ordered by (f, h, node id).
    const heap: number[] = [0];
    const fOf = (id: number) => gw * g[id] + hw * h[id];
    const less = (a: number, b: number) => {
      const fa = fOf(a);
      const fb = fOf(b);
      if (fa !== fb) return fa < fb;
      if (h[a] !== h[b]) return h[a] < h[b];
      return a < b;
    };
    const heapPush = (id: number) => {
      heap.push(id);
      let i = heap.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!less(heap[i], heap[p])) break;
        [heap[i], heap[p]] = [heap[p], heap[i]];
        i = p;
      }
    };
    const heapPop = (): number => {
      const top = heap[0];
      const last = heap.pop() as number;
      if (heap.length > 0) {
        heap[0] = last;
        let i = 0;
        for (;;) {
          const l = i * 2 + 1;
          const r = l + 1;
          let m = i;
          if (l < heap.length && less(heap[l], heap[m])) m = l;
          if (r < heap.length && less(heap[r], heap[m])) m = r;
          if (m === i) break;
          [heap[i], heap[m]] = [heap[m], heap[i]];
          i = m;
        }
      }
      return top;
    };

    const closed = new Map<number, number>();
    const startReach = this.reachable(this.n, new Set(this.startBoxesLocal));
    closed.set(this.pack(this.startBoxesLocal) * 64 + startReach.min, 0);

    let expansions = 0;
    while (heap.length > 0) {
      const id = heapPop();
      if (expansions >= maxExpansions) return null;
      expansions++;
      const boxes = this.unpack(packed[id], nb);
      if (boxes.every((b) => this.goalLocal.has(b))) {
        return {
          goalNode: id,
          nodes: { parent, g, h, player, packed, pushBox, pushDir },
          expansions,
        };
      }
      const boxSet = new Set<number>(boxes);
      const reach = this.reachable(player[id], boxSet);
      for (let bi = 0; bi < nb; bi++) {
        const b = boxes[bi];
        const bg = this.globalOf[b];
        const bx = bg % this.level.w;
        const by = (bg / this.level.w) | 0;
        for (let di = 0; di < 4; di++) {
          const d = DIRS[di];
          // stance: where the player must stand; target: where the box goes
          const sx = bx - d.dx;
          const sy = by - d.dy;
          const tx = bx + d.dx;
          const ty = by + d.dy;
          if (sx < 0 || sy < 0 || sx >= this.level.w || sy >= this.level.h) continue;
          if (tx < 0 || ty < 0 || tx >= this.level.w || ty >= this.level.h) continue;
          const stance = this.localOf.get(sy * this.level.w + sx);
          const target = this.localOf.get(ty * this.level.w + tx);
          if (stance === undefined || target === undefined) continue;
          if (target >= this.n) continue; // never push a box out of the room
          if (boxSet.has(target)) continue;
          if (!reach.set[stance]) continue;
          const newBoxes = [...boxes];
          newBoxes[bi] = target;
          newBoxes.sort((a, b2) => a - b2);
          if (newBoxes.some((c) => this.dead[c] === 1)) continue;
          const newBoxSet = new Set<number>(newBoxes);
          if (this.frozen(newBoxes, newBoxSet)) continue;
          const nh = this.heuristic(newBoxes);
          if (!isFinite(nh)) continue;
          const ng = g[id] + 1;
          const newReach = this.reachable(b, newBoxSet);
          const key = this.pack(newBoxes) * 64 + newReach.min;
          const prevG = closed.get(key);
          if (prevG !== undefined && prevG <= ng) continue;
          closed.set(key, ng);
          const nid = parent.length;
          parent.push(id);
          g.push(ng);
          h.push(nh);
          player.push(b); // player ends on the box's old cell
          packed.push(this.pack(newBoxes));
          pushBox.push(bi);
          pushDir.push(di);
          heapPush(nid);
        }
      }
    }
    return null; // exhausted: unsolvable
  }

  // Rebuild the move string for a goal node. Walks are recomputed with BFS
  // (same fixed neighbour order as everywhere else), so the string only
  // depends on the push sequence, which the search fixed.
  private reconstruct(
    goalNode: number,
    nodes: {
      parent: number[];
      player: number[];
      packed: number[];
      pushBox: number[];
      pushDir: number[];
    },
  ): { moves: string; endPlayer: number; endBoxes: number[]; pushes: number } {
    const nb = this.startBoxesLocal.length;
    const steps: { boxes: number[]; player: number; box: number; dir: number }[] = [];
    let cur = goalNode;
    while (nodes.parent[cur] !== -1) {
      const p = nodes.parent[cur];
      steps.push({
        boxes: this.unpack(nodes.packed[p], nb),
        player: nodes.player[p],
        box: nodes.pushBox[cur],
        dir: nodes.pushDir[cur],
      });
      cur = p;
    }
    steps.reverse();
    let moves = "";
    let curPlayer = this.n; // entry cell
    let curBoxes = [...this.startBoxesLocal];
    for (const st of steps) {
      const boxLocal = st.boxes[st.box];
      const d = DIRS[st.dir];
      const bg = this.globalOf[boxLocal];
      const bx = bg % this.level.w;
      const by = (bg / this.level.w) | 0;
      const stanceG = (by - d.dy) * this.level.w + (bx - d.dx);
      const stance = this.localOf.get(stanceG) as number;
      const walk = this.walkLocal(curPlayer, stance, new Set(curBoxes));
      if (walk === null) throw new Error("reconstruction walk failed");
      moves += walk + d.ch;
      curBoxes = [...st.boxes];
      curBoxes[st.box] = this.localOf.get(
        (by + d.dy) * this.level.w + (bx + d.dx),
      ) as number;
      curBoxes.sort((a, b) => a - b);
      curPlayer = boxLocal;
    }
    return {
      moves,
      endPlayer: this.globalOf[curPlayer],
      endBoxes: curBoxes.map((b) => this.globalOf[b]),
      pushes: steps.length,
    };
  }

  // BFS walk inside the room (+ entry cell), boxes as obstacles.
  private walkLocal(from: number, to: number, boxSet: Set<number>): string | null {
    if (from === to) return "";
    const prev = new Int32Array(this.N).fill(-2);
    const prevCh = new Array<string>(this.N).fill("");
    prev[from] = -1;
    const q = [from];
    let qi = 0;
    while (qi < q.length) {
      const c = q[qi++];
      const g = this.globalOf[c];
      const x = g % this.level.w;
      const y = (g / this.level.w) | 0;
      for (const d of DIRS) {
        const nx = x + d.dx;
        const ny = y + d.dy;
        if (nx < 0 || ny < 0 || nx >= this.level.w || ny >= this.level.h) continue;
        const ni = this.localOf.get(ny * this.level.w + nx);
        if (ni === undefined || boxSet.has(ni) || prev[ni] !== -2) continue;
        prev[ni] = c;
        prevCh[ni] = d.ch;
        if (ni === to) {
          let s = "";
          let cur = ni;
          while (cur !== from) {
            s = prevCh[cur] + s;
            cur = prev[cur];
          }
          return s;
        }
        q.push(ni);
      }
    }
    return null;
  }

  solve(): RoomSolution | null {
    // 1. Optimal A*. 2. Weighted A* (still complete, maybe not optimal).
    // 3. Greedy best-first. All cut off by expansion counts, never by time.
    const attempts: [number, number, number][] = [
      [1, 1, 1500000],
      [1, 4, 800000],
      [0, 1, 800000],
    ];
    for (const [gw, hw, budget] of attempts) {
      const res = this.search(gw, hw, budget);
      this.totalExpansions += res ? res.expansions : budget;
      if (res) {
        const rec = this.reconstruct(res.goalNode, res.nodes);
        return { ...rec, expansions: this.totalExpansions };
      }
    }
    return null;
  }
}

// ---------------------------------------------------------------------------
// Whole-level solver: solve rooms, stitch with hallway walks.
// ---------------------------------------------------------------------------
function solveLevel(level: Level): string {
  const dec = decompose(level);
  if (!dec) return ""; // cannot split: no safe answer
  const rooms = dec.rooms;
  // Solve every room once (independent of order).
  const solved = new Map<Room, RoomSolution>();
  for (const room of rooms) {
    const sol = new RoomSolver(level, room).solve();
    if (!sol) return "";
    solved.set(room, sol);
  }
  // Rooms in door order; try every order until the walks between rooms work
  // (a room whose door ends up blocked by a box on a goal must come last).
  const base = [...rooms].sort((a, b) => a.entry - b.entry);
  const orders = permute(base);
  for (const order of orders) {
    const built = stitch(level, order, solved);
    if (built !== null && replayOk(level, built)) return built;
  }
  return "";
}

function permute<T>(arr: T[]): T[][] {
  if (arr.length <= 1) return [arr];
  const out: T[][] = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = [...arr.slice(0, i), ...arr.slice(i + 1)];
    for (const p of permute(rest)) out.push([arr[i], ...p]);
  }
  return out;
}

function stitch(
  level: Level,
  order: Room[],
  solved: Map<Room, RoomSolution>,
): string | null {
  let moves = "";
  let player = level.player;
  const boxSet = new Set<number>(level.boxes);
  for (const room of order) {
    const sol = solved.get(room) as RoomSolution;
    const toEntry = walkPath(level, player, room.entry, boxSet);
    if (toEntry === null) return null;
    moves += toEntry;
    moves += sol.moves;
    player = sol.endPlayer;
    for (const b of room.boxes) boxSet.delete(b);
    for (const b of sol.endBoxes) boxSet.add(b);
  }
  return moves;
}

// Replay the move string on the level. True only when every step is legal
// and every box ends on a goal.
function replayOk(level: Level, moves: string): boolean {
  const boxes = new Set<number>(level.boxes);
  let player = level.player;
  for (const ch of moves) {
    const d = DIRS.find((dd) => dd.ch === ch);
    if (!d) return false;
    const x = player % level.w;
    const y = (player / level.w) | 0;
    const nx = x + d.dx;
    const ny = y + d.dy;
    if (nx < 0 || ny < 0 || nx >= level.w || ny >= level.h) return false;
    const ni = ny * level.w + nx;
    if (level.wall[ni] !== 0) return false;
    if (boxes.has(ni)) {
      const bx = nx + d.dx;
      const by = ny + d.dy;
      if (bx < 0 || by < 0 || bx >= level.w || by >= level.h) return false;
      const bi = by * level.w + bx;
      if (level.wall[bi] !== 0 || boxes.has(bi)) return false;
      boxes.delete(ni);
      boxes.add(bi);
    }
    player = ni;
  }
  for (const b of boxes) if (!level.goals.has(b)) return false;
  return true;
}

// ---------------------------------------------------------------------------
// Entry point.
// ---------------------------------------------------------------------------
function main(): void {
  // Input/output locations: the harness mounts them at /input and /output in
  // the sandbox, and names them with MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR
  // when it runs the bot on a plain machine (the kit's own entries read them
  // exactly this way). The directories are the only thing read from the
  // environment; nothing about the answers depends on it.
  const inputDir = process.argv[2] ?? process.env.MAKEFUN_INPUT_DIR ?? "/input";
  const outputDir = process.argv[3] ?? process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
  for (const id of ["i001", "i002", "i003", "i004", "i005"]) {
    const inFile = path.join(inputDir, id, "level.txt");
    if (!fs.existsSync(inFile)) continue;
    const level = parseLevel(fs.readFileSync(inFile, "utf8"));
    const moves = solveLevel(level);
    const outDir = path.join(outputDir, id);
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(path.join(outDir, "solution.txt"), moves, "utf8");
  }
}

main();
