# 37 Tools, One MCP Server: The Complete Cognitive Toolkit

*Shodh Memory isn't just storage—it's a full cognitive layer for AI agents.*

---

When we started building [Shodh Memory](/memory), the goal was simple: give AI agents persistent memory. Store things, retrieve things, don't forget between sessions.

But as we used it ourselves, we realized something. Memory isn't enough. A brain doesn't just remember—it organizes, prioritizes, reminds, and plans.

So we kept building. Today, Shodh Memory exposes **37 MCP tools** across five categories. It's not just memory anymore. It's a complete cognitive toolkit.

## The Five Categories

### 1. Memory (10 tools)

The core. Persistent, learning memory with Hebbian associations.

| Tool | What It Does |
|------|--------------|
| `remember` | Store a memory with semantic indexing and entity extraction |
| `recall` | Semantic search across memories (vector + graph hybrid) |
| `recall_by_tags` | Filter memories by tags |
| `recall_by_date` | Find memories within a time range |
| `forget` | Delete a specific memory |
| `forget_by_tags` | Bulk delete by tag |
| `forget_by_date` | Bulk delete by date range |
| `list_memories` | List all stored memories |
| `context_summary` | Get a structured summary of decisions, learnings, errors |
| `proactive_context` | Auto-surface relevant memories for current context |

**The key insight**: `proactive_context` is different from `recall`. You don't search—it surfaces what's relevant based on the current conversation. This mimics how human memory works: associations activate without conscious effort.

```python
# Recall: You ask for something specific
memory.recall("authentication patterns")

# Proactive context: Relevant memories surface automatically
memory.proactive_context("I'm working on the login page")
# Returns: auth decisions, past login work, security patterns
# You didn't ask for these—they're relevant, so they appear
```

### 2. Todos (8 tools)

Full GTD (Getting Things Done) workflow. Not a simple checklist—a complete task management system.

| Tool | What It Does |
|------|--------------|
| `add_todo` | Create a task with project, contexts, priority, due date |
| `list_todos` | Filter by status, project, context, priority, due date |
| `update_todo` | Modify any property of a task |
| `complete_todo` | Mark done (auto-creates next occurrence for recurring tasks) |
| `delete_todo` | Remove a task |
| `reorder_todo` | Change priority ordering within a status group |
| `list_subtasks` | Get child tasks of a parent |
| `todo_stats` | Counts by status, overdue items, completion rates |

**Why this matters for AI agents**: Claude can now track work across sessions. Start a refactoring task today, continue tomorrow. The agent doesn't forget what needs to be done.

```python
# Add a task with GTD contexts
memory.add_todo(
    content="Refactor authentication module",
    project="Backend Cleanup",
    contexts=["@computer", "@deep-work"],
    priority="high",
    due_date="2025-01-15"
)

# Later, list what's actionable right now
memory.list_todos(context="@computer", status=["todo", "in_progress"])
```

### 3. Projects (4 tools)

Organize todos into hierarchical projects. Track progress. Archive when done.

| Tool | What It Does |
|------|--------------|
| `add_project` | Create a project (supports sub-projects via parent) |
| `list_projects` | See all projects with todo counts and status breakdown |
| `archive_project` | Hide completed projects (can be restored) |
| `delete_project` | Permanently remove (optionally with all todos) |

**The codebase indexing feature**: Projects can have an associated codebase path. Index it, and the agent can browse files, understand structure, and work with full project context.

### 4. Reminders (3 tools)

Three trigger types: time-based, duration-based, and context-triggered.

| Tool | What It Does |
|------|--------------|
| `set_reminder` | Create a reminder with trigger conditions |
| `list_reminders` | See pending/triggered/dismissed reminders |
| `dismiss_reminder` | Acknowledge a triggered reminder |

**Context-triggered reminders are the game-changer**:

```python
# This reminder surfaces when you mention "deployment"
memory.set_reminder(
    content="Remember to update the changelog before deploying",
    trigger_type="context",
    keywords=["deployment", "deploy", "release", "ship"]
)

# Days later, you say "Let's deploy to production"
# The reminder automatically surfaces: "Remember to update the changelog..."
```

No calendar needed. The reminder activates when it's relevant.

### 5. System (7 tools)

Introspection, health checks, and maintenance.

| Tool | What It Does |
|------|--------------|
| `memory_stats` | Total memories, retrievals, storage size |
| `verify_index` | Check vector index health, find orphaned memories |
| `repair_index` | Re-index orphaned memories |
| `consolidation_report` | See memory strengthening, decay, edge formation |
| `streaming_status` | Check WebSocket connection health |
| `token_status` | Monitor context window usage |
| `reset_token_session` | Reset token counter for new session |

**The consolidation report** shows what the memory system has learned:

```
Memory Strengthening Events: 47
  - "React patterns" ↔ "Component design" (strength: 0.73, LTP: true)
  - "API design" ↔ "Error handling" (strength: 0.61)

Edge Formation: 12 new associations
Memory Decay: 8 weak connections pruned
Replay Events: 3 important memories reinforced
```

You can literally watch your AI's brain forming connections.

## Why 37 Tools?

The number isn't the point. The point is **completeness**.

Most memory solutions give you store/retrieve. That's a database, not a brain.

A brain:
- Remembers (memory tools)
- Plans and tracks work (todos)
- Organizes into domains (projects)
- Surfaces things at the right time (reminders)
- Has introspection capabilities (system tools)

Shodh Memory provides all of this in a single MCP server. One install, 37 capabilities.

## Real-World Usage

Here's how these tools work together in practice:

### Morning Standup Pattern

```
Claude: Let me check what's on your plate today.

[Calls list_todos with due="today"]
[Calls list_reminders with status="triggered"]
[Calls proactive_context with "morning standup"]

You have 3 tasks due today:
1. Review PR #234 (high priority)
2. Update API documentation
3. Team sync at 2pm

Reminder triggered: "Ask about the deployment timeline in standup"

From memory: Yesterday you were debugging the auth race condition.
Want to continue with that, or tackle the PR review first?
```

### Context Switching

```
You: "Let's work on the mobile app now"

[Calls proactive_context with "mobile app"]

Surfacing relevant context:
- Decision: Using React Native with Expo
- Learning: Android builds need specific Gradle config
- Task in progress: Implement push notifications (60% complete)
- Recent error: iOS simulator crashes with large images

Where would you like to pick up?
```

### End of Session

```
You: "I need to stop for today"

[Calls add_todo for unfinished work]
[Calls remember for session learnings]
[Calls set_reminder for follow-ups]

Captured for next session:
- TODO: Finish implementing the payment webhook
- Remembered: Stripe webhook signatures need raw body parsing
- Reminder: "Test with Stripe CLI before pushing" (triggers on "webhook" or "payment")

See you tomorrow. Your context will be here.
```

## Getting Started

Install Shodh Memory as an MCP server:

```bash
# For Claude Code
claude mcp add shodh-memory -- npx -y @shodh/memory-mcp
```

Or with Python:

```bash
pip install shodh-memory
```

All 37 tools become available immediately. No configuration. No cloud setup. Everything runs locally.

## The Cognitive Layer

LLMs are powerful reasoners but poor rememberers. They can analyze, synthesize, and generate—but they can't retain.

Shodh Memory is the missing layer: a cognitive substrate that handles memory, planning, and organization so the LLM can focus on reasoning.

37 tools. One MCP server. A complete brain.

---

*Try [Shodh Memory](/memory) — the cognitive toolkit for AI agents.*

---

*Tags: MCP, tools, cognitive-architecture, GTD, todos, reminders, AI-agents, productivity*
