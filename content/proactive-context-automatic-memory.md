# Proactive Context: Memory That Surfaces When You Need It

*Stop searching for memories. Let them find you.*

---

There's a fundamental difference between searching for something and having it come to mind.

When you see a friend, you don't query your brain for "memories tagged with John, sorted by recency." Relevant memories just surface—shared experiences, inside jokes, recent conversations. This happens automatically, without conscious effort.

Most AI memory systems work like databases: you query, they return results. [Shodh Memory](/memory) works differently. The `proactive_context` tool surfaces relevant memories automatically based on the current conversation.

This is how biological memory works. And it changes everything.

## Search vs. Recall

### Traditional Memory Search

```python
# You explicitly search
results = memory.recall("authentication patterns")

# You get back what matches your query
# But you had to know what to ask for
```

This works when you know what you need. But often, you don't. The most valuable context is the thing you forgot to ask about.

### Proactive Context

```python
# You provide the current conversation context
memories = memory.proactive_context("I'm implementing the login page")

# The system surfaces what's relevant:
# - "We decided to use JWT with refresh tokens" (Decision)
# - "Last login implementation had a race condition with state updates" (Error)
# - "User prefers shadcn/ui for auth forms" (Preference)
# - "OAuth integration is planned for v2" (Context)
```

You didn't ask for these. They're relevant, so they appear.

## How It Works

Proactive context uses three signals to determine relevance:

### 1. Entity Matching (40% weight)

The system extracts entities from your current context and finds memories mentioning the same entities.

```
Context: "Working on the Stripe payment integration"

Entities extracted: ["Stripe", "payment", "integration"]

Memories mentioning these entities get boosted:
- "Stripe webhooks require raw body parsing" ✓
- "Payment flow needs idempotency keys" ✓
- "Integration tests run on CI" (different "integration") △
```

This isn't keyword matching—it's entity recognition. "Payment" matches memories about "payments," "pay," and "billing" because they're semantically linked.

### 2. Semantic Similarity (40% weight)

Vector similarity between your context and stored memories:

```
Context embedding: [0.12, -0.34, 0.78, ...]

Memory: "JWT tokens expire after 24 hours"
Similarity: 0.23 (low - not about payments)

Memory: "Stripe test mode uses different API keys"
Similarity: 0.87 (high - about Stripe)

Memory: "Always verify webhook signatures"
Similarity: 0.72 (medium-high - about integrations)
```

Memories above the threshold (default 0.65) are included.

### 3. Recency Boost (20% weight)

Recent memories get a boost. Yesterday's decision is more relevant than last year's.

```
Memory from today: 1.0x boost
Memory from this week: 0.8x boost
Memory from this month: 0.5x boost
Memory from last year: 0.1x boost
```

This mimics biological memory where recent events are more accessible.

## The Combined Score

```
relevance = (entity_match × 0.4) + (semantic_sim × 0.4) + (recency × 0.2)
```

Memories are ranked by this score. The top N (default 5) are returned.

## Why This Matters

### 1. You Can't Search for What You Forgot

The biggest problem with explicit search is that you need to know what to ask. But the most valuable context is often something you've forgotten.

```
# You're about to implement caching
# You forgot that 3 months ago, you decided against Redis

Proactive context for "adding cache layer":
→ "Decision: Use in-memory caching initially, Redis for v2"

# The decision surfaces automatically
# You don't repeat past mistakes
```

### 2. Context Flows Naturally

In a conversation, relevant memories should appear as the topic shifts—without explicit queries.

```
You: "Let's work on the dashboard"
→ Dashboard-related memories surface

You: "The charts are slow"
→ Performance optimization memories surface

You: "Maybe we need to paginate the data"
→ Past pagination decisions surface
```

Each message updates the context, and relevant memories flow in naturally.

### 3. Less Cognitive Load

With explicit search, you have to:
1. Recognize you need information
2. Formulate a query
3. Review results
4. Repeat if not found

With proactive context:
1. Relevant information appears automatically

The difference is like GPS navigation vs. memorizing directions.

## Using Proactive Context

### Basic Usage

```python
from shodh_memory import Memory

memory = Memory()

# Provide the current conversation or task
context = memory.proactive_context(
    "I'm debugging the authentication flow. Users report being logged out randomly."
)

# Returns the most relevant memories
for mem in context:
    print(f"[{mem['memory_type']}] {mem['content']}")
```

### Configuring Sensitivity

```python
# More strict - only highly relevant memories
context = memory.proactive_context(
    "current task description",
    semantic_threshold=0.75,  # Higher = more strict
    max_results=3
)

# More permissive - cast a wider net
context = memory.proactive_context(
    "current task description",
    semantic_threshold=0.5,  # Lower = more permissive
    max_results=10
)
```

### Filtering by Memory Type

```python
# Only surface decisions and errors (not general context)
context = memory.proactive_context(
    "current task description",
    memory_types=["Decision", "Error", "Learning"]
)
```

### Adjusting Weights

```python
# Prioritize entity matching over semantic similarity
context = memory.proactive_context(
    "current task description",
    entity_match_weight=0.6,  # Up from 0.4
    recency_weight=0.1        # Down from 0.2
)
```

## Auto-Ingest Mode

By default, `proactive_context` also stores the current context as a Conversation memory:

```python
# This both retrieves relevant memories AND stores the context
context = memory.proactive_context(
    "Working on payment integration",
    auto_ingest=True  # Default
)

# Now "Working on payment integration" is stored as a memory
# Future calls can surface it as relevant context
```

This creates a virtuous cycle: your conversations become part of the memory, informing future context surfacing.

## MCP Integration

In Claude Code, `proactive_context` is called automatically at session start and can be called throughout the conversation:

```json
{
  "mcpServers": {
    "shodh-memory": {
      "command": "npx",
      "args": ["-y", "@shodh/memory-mcp"]
    }
  }
}
```

When Claude starts a session:

```
[proactive_context called with session start hook]

Surfacing relevant context:
- Last session: Working on user authentication
- Pending task: Implement password reset flow
- Decision: Using SendGrid for transactional email
- Reminder triggered: "Test email templates before pushing"

Claude: Welcome back! Last time you were implementing the password reset
flow. You decided to use SendGrid for emails. Ready to continue?
```

No explicit recall needed. Context flows in.

## The Biological Inspiration

Human memory doesn't have a search bar. When you think of your childhood home, associated memories surface: the smell of the kitchen, the creak of the stairs, family dinners. You didn't query for these—they activated through association.

This is called **spreading activation** in cognitive science. Activating one memory spreads activation to connected memories. The most activated memories surface to consciousness.

`proactive_context` implements this:

1. Your current context activates matching entities and embeddings
2. Activation spreads through the knowledge graph
3. The most activated memories surface

It's not a search algorithm. It's a memory system.

## When to Use What

| Scenario | Tool |
|----------|------|
| Know exactly what you need | `recall` |
| Need memories by tag | `recall_by_tags` |
| Need memories from a period | `recall_by_date` |
| Want relevant context to flow in | `proactive_context` |
| Starting a work session | `proactive_context` |
| Context-shifting to new topic | `proactive_context` |

Most AI-agent workflows should use `proactive_context` as the primary retrieval method, falling back to explicit `recall` when searching for something specific.

---

*Stop searching. Start remembering. Try [Shodh Memory](/memory).*

---

*Tags: proactive-context, spreading-activation, memory-retrieval, AI-agents, cognitive-science, MCP*
