# The TUI Dashboard: Watch Your AI's Brain Form Memories

*Introspection for AI memory. See Hebbian learning in real-time.*

---

AI systems are often black boxes. You put data in, results come out, and what happens in between is a mystery.

[Shodh Memory](/memory) includes something different: a Terminal User Interface (TUI) that lets you watch your AI's memory system in real-time. See memories form. Watch associations strengthen. Browse the knowledge graph. Explore indexed codebases.

It's introspection for AI—and it's surprisingly useful.

![Shodh Memory TUI Dashboard](/blog/splash.jpg)

## Why a TUI?

Three reasons:

**1. Debugging**: When the AI doesn't recall something you expect, you can see why. Is the memory stored? Is the association weak? Was it decayed?

**2. Understanding**: Watching Hebbian strengthening in real-time builds intuition about how the system learns. You start to understand *why* certain memories surface together.

**3. Confidence**: Seeing the memory system work—edges forming, importance scores updating, consolidation happening—builds trust in the system.

Plus, it looks cool.

## The Main Dashboard

The dashboard is your command center for memory introspection:

![Main Dashboard View](/blog/dashboard.jpg)

The dashboard shows:

- **Memory List**: All stored memories, grouped by type (Learning, Decision, Error, Context, etc.)
- **Recent Activity**: Real-time log of memory operations
- **Stats**: Quick metrics on memory health—total memories, edges, LTP connections, storage size
- **Session Context**: What's currently active across different users/sessions

### Real-Time Activity Feed

Watch operations as they happen:

```
14:23:01  REMEMBER   "JWT refresh token implementation requires..."
14:23:01  EMBED      384-dim vector generated (47ms)
14:23:01  NER        Entities: [JWT, refresh token, implementation]
14:23:01  EDGE       New edge: jwt ↔ authentication (strength: 0.15)
14:23:01  EDGE       Strengthened: auth ↔ security (+0.12 → 0.67)
```

You can see:
- Memory creation with embedding time
- Entity extraction from NER
- Edge formation and strengthening
- Importance score updates

### Hebbian Learning Visualization

When memories are retrieved together, their connection strengthens:

```
RECALL query: "authentication security"
  Retrieved: [JWT tokens, Session handling, Password hashing]

  Edge updates:
    jwt ↔ session:    0.45 → 0.52 (+0.07)
    jwt ↔ password:   0.23 → 0.31 (+0.08)
    session ↔ password: 0.67 → 0.72 (+0.05)

  Co-activation count: jwt↔session = 8 (LTP threshold: 10)
```

After enough co-activations, connections become permanent (Long-Term Potentiation):

```
14:45:22  LTP        jwt ↔ session (activation: 10, permanent: true)
                     This connection will now decay 10x slower
```

## Knowledge Graph Explorer

Navigate the association graph visually:

![Knowledge Graph Map](/blog/graph-map.jpg)

The graph view shows:

- **Nodes**: Entities and memory clusters
- **Edges**: Connections with strength indicated by thickness
- **LTP Edges**: Permanent connections highlighted
- **Activation**: Recently accessed nodes glow

You can:
- Navigate with vim keys (h/j/k/l)
- Expand nodes to see connected memories
- Search for specific entities
- Filter by edge strength or LTP status

## Projects and Todos View

The TUI also provides a complete view of your task management:

![Projects and Todos](/blog/projects-todos.jpg)

This view shows:

- **Projects**: Hierarchical project structure with sub-projects
- **Todos**: Tasks organized by status (backlog, todo, in_progress, done)
- **Codebase Status**: Which projects have indexed codebases
- **File Browser**: Navigate indexed files with related memories

### Codebase Indexing

For projects with indexed codebases, you can:

- Browse the file tree
- See which files have associated memories
- View file contents directly in the TUI
- Jump to related memories for any file

```
📁 src/
  📁 controllers/
    📄 auth.controller.ts      ← 8 memories reference this
    📄 user.controller.ts      ← 3 memories
  📁 services/
    📄 auth.service.ts         ← 12 memories reference this
```

## Session Context View

See what context is active across different sessions:

```
┌─ ACTIVE SESSIONS ───────────────────────────────────────────────────┐
│                                                                     │
│  ● claude-code (active now)                                        │
│    Current context: "implementing payment webhooks"                 │
│    Memories surfaced: 7                                            │
│    Session duration: 1h 23m                                        │
│                                                                     │
│  ○ cursor-ide (idle 3h)                                            │
│    Last context: "frontend component refactoring"                  │
│    Memories created: 12                                            │
│                                                                     │
│  ○ python-agent (idle 2d)                                          │
│    Last context: "data pipeline optimization"                      │
│    Memories created: 45                                            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Consolidation Report

See what the memory system has learned over time:

```
┌─ CONSOLIDATION REPORT (Last 24h) ───────────────────────────────────┐
│                                                                     │
│  Memory Strengthening: 47 events                                   │
│    Strongest new association: auth ↔ jwt (0.87)                    │
│    Most reinforced: react ↔ components (+0.34 total)               │
│                                                                     │
│  Long-Term Potentiation: 3 new permanent connections               │
│    • auth ↔ session (activated 12 times)                           │
│    • prisma ↔ database (activated 10 times)                        │
│    • api ↔ endpoints (activated 11 times)                          │
│                                                                     │
│  Decay Events: 12 weak edges pruned                                │
│    • old-framework ↔ deprecated (unused 30d)                       │
│    • temp-fix ↔ workaround (unused 45d)                           │
│                                                                     │
│  Memory Replay: 5 important memories reinforced                    │
│    • "Production database credentials" (importance: 0.95)          │
│    • "API rate limiting configuration" (importance: 0.88)          │
│                                                                     │
│  Health: ✓ Good                                                    │
│    Index integrity: 100%                                           │
│    Orphaned memories: 0                                            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Running the TUI

```bash
# Install the TUI binary
cargo install shodh-memory-tui

# Or download the release binary
# Run it
shodh-tui

# Connect to a specific server
shodh-tui --server http://localhost:3030

# Watch a specific user's memories
shodh-tui --user claude-code
```

### Keyboard Navigation

| Key | Action |
|-----|--------|
| `Tab` | Switch panels |
| `j/k` | Navigate up/down |
| `h/l` | Navigate left/right (graph view) |
| `Enter` | Select/expand |
| `/` | Search |
| `r` | Refresh |
| `g` | Go to graph view |
| `p` | Go to projects view |
| `m` | Go to memories view |
| `q` | Back/quit |
| `?` | Help |

## Debugging with the TUI

### "Why didn't Claude recall this memory?"

1. Open the TUI, find the memory in the list
2. Check its importance score (too low?)
3. Check its last access time (decayed?)
4. Check connected edges (weak associations?)
5. Check entity overlap with query (no matching entities?)

### "Why are these memories connected?"

1. Open the knowledge graph view
2. Find the edge between them
3. Check activation count (how many times co-retrieved?)
4. Check if LTP (permanent or temporary?)

### "What has the system learned?"

1. Open the consolidation report
2. Review LTP events (permanent connections)
3. Review edge strengthening (emerging patterns)
4. Review decay events (what's fading)

## The Value of Transparency

AI memory systems are only useful if you can trust them. Black-box storage that might or might not recall things correctly isn't helpful.

The TUI provides transparency:
- See exactly what's stored
- Watch learning happen in real-time
- Debug retrieval issues visually
- Understand the system's behavior

This isn't just a debugging tool. It's a trust-building tool. When you can watch the AI's brain form connections, you understand why it behaves the way it does.

---

*See your AI's brain in action. Try [Shodh Memory](/memory).*

---

*Tags: TUI, dashboard, debugging, introspection, Hebbian-learning, visualization, developer-tools*
