'use client';

import { useParams } from 'next/navigation';
import Link from 'next/link';
import { motion } from 'framer-motion';
import { ArrowLeft, Calendar, Clock, Tag } from 'lucide-react';
import { getPostBySlug, getAllPosts } from '@/lib/posts';
import { formatDate, getCategoryColor } from '@/lib/blog';
import CodeBlock from '@/components/CodeBlock';

function ArticleStructuredData({ post }: { post: ReturnType<typeof getPostBySlug> }) {
  if (!post) return null;

  const articleSchema = {
    '@context': 'https://schema.org',
    '@type': 'TechArticle',
    headline: post.title,
    description: post.description,
    image: 'https://shodh-rag.com/og-image.png',
    datePublished: post.date,
    dateModified: post.date,
    author: {
      '@type': 'Organization',
      name: post.author.name,
      url: 'https://shodh-rag.com',
    },
    publisher: {
      '@type': 'Organization',
      name: 'Shodh',
      logo: {
        '@type': 'ImageObject',
        url: 'https://shodh-rag.com/logo.png',
      },
    },
    mainEntityOfPage: {
      '@type': 'WebPage',
      '@id': `https://shodh-rag.com/blog/${post.slug}`,
    },
    keywords: post.tags.join(', '),
    articleSection: post.category,
    wordCount: post.readingTime.includes('min')
      ? parseInt(post.readingTime) * 200
      : 2000,
    proficiencyLevel: 'Beginner',
  };

  const breadcrumbSchema = {
    '@context': 'https://schema.org',
    '@type': 'BreadcrumbList',
    itemListElement: [
      {
        '@type': 'ListItem',
        position: 1,
        name: 'Home',
        item: 'https://shodh-rag.com',
      },
      {
        '@type': 'ListItem',
        position: 2,
        name: 'Blog',
        item: 'https://shodh-rag.com/blog',
      },
      {
        '@type': 'ListItem',
        position: 3,
        name: post.title,
        item: `https://shodh-rag.com/blog/${post.slug}`,
      },
    ],
  };

  const howToSchema = post.category === 'Tutorial' ? {
    '@context': 'https://schema.org',
    '@type': 'HowTo',
    name: post.title,
    description: post.description,
    image: 'https://shodh-rag.com/og-image.png',
    totalTime: post.readingTime,
    tool: [
      { '@type': 'HowToTool', name: 'Claude Code CLI' },
      { '@type': 'HowToTool', name: 'Node.js 18+' },
      { '@type': 'HowToTool', name: 'Terminal' },
    ],
    step: [
      {
        '@type': 'HowToStep',
        name: 'Install Prerequisites',
        text: 'Install Claude Code and Node.js 18+',
      },
      {
        '@type': 'HowToStep',
        name: 'Configure MCP Server',
        text: 'Add MCP server to settings.json or use claude mcp add command',
      },
      {
        '@type': 'HowToStep',
        name: 'Start the Server',
        text: 'Start your MCP server (e.g., Shodh Memory server)',
      },
      {
        '@type': 'HowToStep',
        name: 'Restart Claude Code',
        text: 'Restart Claude Code to load the new MCP server',
      },
      {
        '@type': 'HowToStep',
        name: 'Test the Tools',
        text: 'Verify the MCP tools are available and working',
      },
    ],
  } : null;

  return (
    <>
      <script
        type="application/ld+json"
        dangerouslySetInnerHTML={{ __html: JSON.stringify(articleSchema) }}
      />
      <script
        type="application/ld+json"
        dangerouslySetInnerHTML={{ __html: JSON.stringify(breadcrumbSchema) }}
      />
      {howToSchema && (
        <script
          type="application/ld+json"
          dangerouslySetInnerHTML={{ __html: JSON.stringify(howToSchema) }}
        />
      )}
    </>
  );
}

// Blog post content components
function ClaudeCodeMCPSetupPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        Claude Code is Anthropic's powerful CLI tool that brings Claude directly into your terminal.
        But its true power unlocks when you connect it to MCP (Model Context Protocol) servers—giving
        Claude access to external tools, databases, and services. This guide walks you through
        setting up MCP servers from scratch, with a practical example using Shodh Memory for
        persistent AI memory.
      </p>

      <h2>What is MCP (Model Context Protocol)?</h2>
      <p>
        MCP is an open protocol that allows AI assistants like Claude to securely connect to
        external data sources and tools. Think of it as a USB standard for AI—a universal way
        to plug capabilities into your AI assistant.
      </p>
      <p>
        With MCP servers, Claude Code can:
      </p>
      <ul>
        <li><strong>Remember across sessions</strong> — Store and recall information persistently</li>
        <li><strong>Access databases</strong> — Query PostgreSQL, MongoDB, or custom data stores</li>
        <li><strong>Integrate with APIs</strong> — Connect to GitHub, Linear, Slack, and more</li>
        <li><strong>Execute specialized tools</strong> — Run domain-specific operations</li>
      </ul>

      <h2>Prerequisites</h2>
      <p>Before starting, ensure you have:</p>
      <ul>
        <li>Claude Code installed (<code>npm install -g @anthropic-ai/claude-code</code>)</li>
        <li>Node.js 18+ (for running MCP servers)</li>
        <li>A working terminal (macOS, Linux, or Windows with WSL)</li>
      </ul>

      <h2>Understanding MCP Configuration</h2>
      <p>
        Claude Code uses a JSON configuration file to know which MCP servers to connect to.
        The location depends on your platform:
      </p>

      <CodeBlock
        language="text"
        filename="Configuration Locations"
        code={`# macOS
~/.config/claude-code/settings.json

# Linux
~/.config/claude-code/settings.json

# Windows
%APPDATA%\\claude-code\\settings.json

# Project-specific (any platform)
./.mcp.json  (in your project root)`}
      />

      <h2>Method 1: Using the CLI (Recommended)</h2>
      <p>
        The easiest way to add MCP servers is through Claude Code's built-in commands:
      </p>

      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`# Add an MCP server from npm
claude mcp add @shodh/memory-mcp

# Add with custom configuration
claude mcp add @shodh/memory-mcp --env SHODH_API_URL=http://localhost:3030

# List configured servers
claude mcp list

# Remove a server
claude mcp remove @shodh/memory-mcp`}
      />

      <p>
        When you run <code>claude mcp add</code>, Claude Code automatically:
      </p>
      <ol>
        <li>Creates the configuration file if it doesn't exist</li>
        <li>Adds the server entry with the correct format</li>
        <li>Validates the server can be started</li>
      </ol>

      <h2>Method 2: Manual Configuration</h2>
      <p>
        For more control, you can edit the configuration file directly. Here's the
        complete structure:
      </p>

      <CodeBlock
        language="json"
        filename="settings.json"
        code={`{
  "mcpServers": {
    "shodh-memory": {
      "command": "npx",
      "args": ["-y", "@shodh/memory-mcp"],
      "env": {
        "SHODH_API_URL": "http://127.0.0.1:3030",
        "SHODH_API_KEY": "your-api-key-here",
        "SHODH_USER_ID": "claude-code"
      }
    },
    "github": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-github"],
      "env": {
        "GITHUB_PERSONAL_ACCESS_TOKEN": "ghp_xxxxxxxxxxxx"
      }
    },
    "filesystem": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-filesystem", "/path/to/allowed/dir"]
    }
  }
}`}
      />

      <h3>Configuration Fields Explained</h3>
      <table>
        <thead>
          <tr><th>Field</th><th>Required</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td><code>command</code></td><td>Yes</td><td>Executable to run (npx, node, python, etc.)</td></tr>
          <tr><td><code>args</code></td><td>Yes</td><td>Arguments passed to the command</td></tr>
          <tr><td><code>env</code></td><td>No</td><td>Environment variables for the server process</td></tr>
          <tr><td><code>cwd</code></td><td>No</td><td>Working directory for the server</td></tr>
        </tbody>
      </table>

      <h2>Method 3: Project-Specific Configuration</h2>
      <p>
        For project-specific MCP servers, create a <code>.mcp.json</code> file in your
        project root:
      </p>

      <CodeBlock
        language="json"
        filename=".mcp.json"
        code={`{
  "mcpServers": {
    "project-memory": {
      "command": "npx",
      "args": ["-y", "@shodh/memory-mcp"],
      "env": {
        "SHODH_API_URL": "http://127.0.0.1:3030",
        "SHODH_USER_ID": "my-project"
      }
    }
  }
}`}
      />

      <p>
        Project-specific servers are only available when Claude Code is running in that
        directory. This is useful for:
      </p>
      <ul>
        <li>Project-specific databases or APIs</li>
        <li>Different configurations per project</li>
        <li>Sharing MCP setup with your team (commit <code>.mcp.json</code> to git)</li>
      </ul>

      <h2>Setting Up Shodh Memory Server</h2>
      <p>
        Let's walk through a complete example: adding persistent memory to Claude Code
        using Shodh Memory. This gives Claude the ability to remember information across
        sessions—preferences, project context, decisions, and learnings.
      </p>

      <h3>Step 1: Start the Shodh Memory Server</h3>
      <p>
        First, download and run the Shodh Memory server. You can use the pre-built
        binary or run from source:
      </p>

      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`# Option A: Using pre-built binary (recommended)
# Download from: https://github.com/varun29ankuS/shodh-memory/releases

# macOS/Linux
chmod +x shodh-memory-server
./shodh-memory-server

# Windows
shodh-memory-server.exe

# Option B: From source
git clone https://github.com/varun29ankuS/shodh-memory
cd shodh-memory
cargo run --release

# Server starts on http://127.0.0.1:3030 by default`}
      />

      <p>
        Verify the server is running:
      </p>

      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`curl http://127.0.0.1:3030/health
# Response: {"status":"ok"}`}
      />

      <h3>Step 2: Add the MCP Server to Claude Code</h3>

      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`# Using the CLI
claude mcp add @shodh/memory-mcp \\
  --env SHODH_API_URL=http://127.0.0.1:3030 \\
  --env SHODH_API_KEY=your-api-key \\
  --env SHODH_USER_ID=claude-code`}
      />

      <p>Or manually add to your <code>settings.json</code>:</p>

      <CodeBlock
        language="json"
        filename="settings.json"
        code={`{
  "mcpServers": {
    "shodh-memory": {
      "command": "npx",
      "args": ["-y", "@shodh/memory-mcp"],
      "env": {
        "SHODH_API_URL": "http://127.0.0.1:3030",
        "SHODH_API_KEY": "sk-shodh-dev-4f8b2c1d9e3a7f5b6d2c8e4a1b9f7d3c",
        "SHODH_USER_ID": "claude-code"
      }
    }
  }
}`}
      />

      <h3>Step 3: Restart Claude Code</h3>
      <p>
        After configuring, restart Claude Code to load the new MCP server:
      </p>

      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`# Exit Claude Code (Ctrl+C or type /exit)
# Then restart
claude`}
      />

      <h3>Step 4: Test the Memory Tools</h3>
      <p>
        Once connected, Claude has access to memory tools. Try these commands:
      </p>

      <CodeBlock
        language="text"
        filename="Claude Code"
        code={`> Remember that I prefer dark mode and use Vim keybindings
[Claude uses the remember tool to store this preference]

> What are my preferences?
[Claude uses the recall tool to search memories]

> Give me a summary of what you know about me
[Claude uses context_summary to get categorized memories]`}
      />

      <h2>Available Memory Tools</h2>
      <p>
        The Shodh Memory MCP server exposes these tools to Claude:
      </p>

      <table>
        <thead>
          <tr><th>Tool</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td><code>remember</code></td><td>Store a new memory with content, type, and optional tags</td></tr>
          <tr><td><code>recall</code></td><td>Semantic search for relevant memories</td></tr>
          <tr><td><code>context_summary</code></td><td>Get categorized summary of recent learnings and decisions</td></tr>
          <tr><td><code>list_memories</code></td><td>List all stored memories</td></tr>
          <tr><td><code>forget</code></td><td>Delete a specific memory by ID</td></tr>
          <tr><td><code>memory_stats</code></td><td>Get statistics about stored memories</td></tr>
          <tr><td><code>recall_by_tags</code></td><td>Search memories by tags</td></tr>
          <tr><td><code>recall_by_date</code></td><td>Search memories within a date range</td></tr>
          <tr><td><code>forget_by_tags</code></td><td>Delete memories matching tags</td></tr>
          <tr><td><code>forget_by_date</code></td><td>Delete memories within a date range</td></tr>
        </tbody>
      </table>

      <h2>Memory Types for Better Organization</h2>
      <p>
        When storing memories, you can specify types to help with retrieval and
        prioritization:
      </p>

      <CodeBlock
        language="text"
        filename="Memory Types"
        code={`Observation  — General observations and facts
Decision     — Choices made and their rationale
Learning     — New knowledge acquired
Error        — Mistakes to avoid
Discovery    — Findings from exploration
Pattern      — Recurring behaviors identified
Context      — Session or project context
Task         — Work items and todos
CodeEdit     — Code changes made
FileAccess   — Files accessed
Search       — Search queries performed
Command      — Commands executed
Conversation — Notable conversation points`}
      />

      <h2>Practical Examples</h2>

      <h3>Example 1: Project Onboarding</h3>
      <p>
        When starting work on a new project, Claude can build up context that persists:
      </p>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`> Explore this codebase and remember the key architectural decisions

[Claude explores and stores memories like:]
- "This project uses Next.js 14 with App Router"
- "Authentication is handled by NextAuth with GitHub OAuth"
- "Database is PostgreSQL accessed via Prisma ORM"
- "Tests use Vitest with React Testing Library"

# Next session, same project:
> What's the tech stack here?

[Claude recalls from memory without re-exploring]`}
      />

      <h3>Example 2: User Preferences</h3>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`> Remember: I prefer functional components over class components,
> use TypeScript strict mode, and always add JSDoc comments to public APIs

[Claude stores as Decision type memories]

# Later, when writing code:
> Create a new React component for user profiles

[Claude automatically applies remembered preferences]`}
      />

      <h3>Example 3: Learning from Errors</h3>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`> The tests are failing with "Cannot find module '@/lib/auth'"

[After investigation]
> Remember: path aliases in this project require tsconfig paths
> to be mirrored in vitest.config.ts

[Claude stores as Error type memory]

# Future similar issues are caught faster`}
      />

      <h2>Troubleshooting Common Issues</h2>

      <h3>Server Connection Failed</h3>
      <CodeBlock
        language="bash"
        filename="Debugging"
        code={`# Check if server is running
curl http://127.0.0.1:3030/health

# Check Claude Code logs
claude --verbose

# Verify configuration
cat ~/.config/claude-code/settings.json | jq .`}
      />

      <h3>Tools Not Appearing</h3>
      <p>
        If MCP tools don't show up in Claude:
      </p>
      <ol>
        <li>Ensure the server is running before starting Claude Code</li>
        <li>Check for typos in the configuration file</li>
        <li>Verify the <code>args</code> array is correctly formatted</li>
        <li>Look for error messages in Claude Code's output</li>
      </ol>

      <h3>Permission Errors</h3>
      <CodeBlock
        language="bash"
        filename="Fix Permissions"
        code={`# Ensure npx can run the package
npx -y @shodh/memory-mcp --help

# If using a local server, check firewall rules
# On macOS, you may need to allow incoming connections`}
      />

      <h2>Security Best Practices</h2>
      <ul>
        <li><strong>Use environment variables</strong> for API keys, never hardcode them</li>
        <li><strong>Limit server scope</strong> — only expose necessary tools and data</li>
        <li><strong>Run locally when possible</strong> — avoid sending sensitive data to external servers</li>
        <li><strong>Review MCP server code</strong> before using third-party servers</li>
        <li><strong>Use project-specific configs</strong> to isolate different projects</li>
      </ul>

      <h2>Popular MCP Servers</h2>
      <p>
        Beyond Shodh Memory, here are other useful MCP servers:
      </p>

      <table>
        <thead>
          <tr><th>Server</th><th>Package</th><th>Use Case</th></tr>
        </thead>
        <tbody>
          <tr><td>GitHub</td><td><code>@modelcontextprotocol/server-github</code></td><td>Repository operations, issues, PRs</td></tr>
          <tr><td>Filesystem</td><td><code>@modelcontextprotocol/server-filesystem</code></td><td>Read/write files (sandboxed)</td></tr>
          <tr><td>PostgreSQL</td><td><code>@modelcontextprotocol/server-postgres</code></td><td>Database queries</td></tr>
          <tr><td>Brave Search</td><td><code>@modelcontextprotocol/server-brave-search</code></td><td>Web search</td></tr>
          <tr><td>Puppeteer</td><td><code>@modelcontextprotocol/server-puppeteer</code></td><td>Browser automation</td></tr>
          <tr><td>Linear</td><td><code>@linear/mcp-server</code></td><td>Issue tracking</td></tr>
        </tbody>
      </table>

      <h2>Building Your Own MCP Server</h2>
      <p>
        MCP servers can be built in any language that supports JSON-RPC over stdio.
        The official SDK is available for TypeScript/Node.js:
      </p>

      <CodeBlock
        language="typescript"
        filename="Custom MCP Server"
        code={`import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";

const server = new Server({
  name: "my-custom-server",
  version: "1.0.0",
}, {
  capabilities: {
    tools: {},
  },
});

// Define a tool
server.setRequestHandler("tools/list", async () => ({
  tools: [{
    name: "my_tool",
    description: "Does something useful",
    inputSchema: {
      type: "object",
      properties: {
        query: { type: "string", description: "The input" }
      },
      required: ["query"]
    }
  }]
}));

// Handle tool calls
server.setRequestHandler("tools/call", async (request) => {
  if (request.params.name === "my_tool") {
    const query = request.params.arguments.query;
    return { content: [{ type: "text", text: \`Processed: \${query}\` }] };
  }
});

// Start server
const transport = new StdioServerTransport();
await server.connect(transport);`}
      />

      <h2>Conclusion</h2>
      <p>
        MCP servers transform Claude Code from a powerful assistant into an extensible
        platform. With persistent memory via Shodh Memory, Claude remembers your
        preferences, learns from experience, and maintains context across sessions—just
        like a human colleague would.
      </p>
      <p>
        The setup takes less than 5 minutes, but the productivity gains compound over
        time. Every preference remembered, every pattern learned, every error avoided
        adds up.
      </p>
      <p>
        Start with the basics: add Shodh Memory for persistent context. Then explore
        other MCP servers for GitHub integration, database access, or build your own
        for domain-specific needs.
      </p>
    </article>
  );
}

function AgenticAIMemoryPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        Most AI agents have amnesia. They process a request, respond, and forget everything.
        The next session starts from scratch. This is the "context window problem" that
        limits what autonomous agents can actually accomplish.
      </p>

      <h2>The Problem: Context Windows Are Finite</h2>
      <p>
        LLMs operate within fixed context windows (4K-128K tokens). Long-running agents
        face "context bloat" - the window fills with tool outputs, previous reasoning,
        and conversation history. At some point, you hit the limit and have to drop context.
      </p>
      <p>
        The typical solution is summarization: compress old context into shorter summaries.
        But summarization loses detail. The agent forgets the specific error message that
        caused a failure, the exact configuration that worked, the reasoning behind a decision.
      </p>

      <h2>How Biological Memory Actually Works</h2>
      <p>
        Human memory doesn't work like a context window. It operates through:
      </p>
      <ul>
        <li><strong>Hebbian learning</strong>: "Neurons that fire together, wire together." Associations strengthen with repeated co-activation.</li>
        <li><strong>Activation decay</strong>: Unused memories fade over time following the Ebbinghaus forgetting curve.</li>
        <li><strong>Long-Term Potentiation (LTP)</strong>: After sufficient repetition, synaptic connections become permanent.</li>
        <li><strong>Consolidation</strong>: Episodic memories (events) transform into semantic memories (facts) during sleep.</li>
      </ul>
      <p>
        These mechanisms evolved because they're efficient. You don't need to remember
        every detail of every day - you need to remember what matters.
      </p>

      <h2>Implementing Hebbian Learning in Code</h2>
      <p>
        Here's the core idea: when two memories are retrieved together, strengthen
        the connection between them. When a memory isn't accessed, let it decay.
      </p>

      <CodeBlock
        language="rust"
        filename="graph_memory.rs"
        code={`/// Hebbian synaptic plasticity: "Neurons that fire together, wire together"
/// - Strength increases with co-activation
/// - Strength decays over time without use
/// - After threshold activations, becomes permanent (LTP)

const LEARNING_RATE: f32 = 0.1;
const LTP_THRESHOLD: u32 = 5;
const LTP_DECAY_FACTOR: f32 = 0.1; // Potentiated synapses decay 10x slower

impl Relationship {
    pub fn strengthen(&mut self) {
        self.activation_count += 1;

        // Hebbian strengthening: diminishing returns as strength approaches 1.0
        let delta = LEARNING_RATE * (1.0 - self.strength);
        self.strength = (self.strength + delta).min(1.0);

        // Check for Long-Term Potentiation
        if !self.potentiated && self.activation_count >= LTP_THRESHOLD {
            self.potentiated = true;
        }
    }

    pub fn decay(&mut self) -> bool {
        let days_elapsed = (Utc::now() - self.last_accessed).num_hours() as f64 / 24.0;
        let half_life = BASE_HALF_LIFE_HOURS * (1.0 + self.strength) as f64;

        // Potentiated synapses decay much slower
        let effective_half_life = if self.potentiated {
            half_life / LTP_DECAY_FACTOR as f64
        } else {
            half_life
        };

        let decay_factor = (-0.693 / effective_half_life * days_elapsed).exp() as f32;
        self.strength *= decay_factor;

        self.strength < MIN_STRENGTH // Return true if should be pruned
    }
}`}
      />

      <h2>Three-Tier Memory Hierarchy</h2>
      <p>
        Raw storage isn't enough. You need different tiers for different access patterns:
      </p>
      <ul>
        <li><strong>Working memory</strong>: Fast access, limited capacity (like CPU cache). Current context and active associations.</li>
        <li><strong>Session memory</strong>: Medium-term storage for the current task. Survives context window resets.</li>
        <li><strong>Long-term memory</strong>: Persistent storage in RocksDB. Compressed, indexed by semantic similarity.</li>
      </ul>

      <CodeBlock
        language="rust"
        filename="mod.rs"
        code={`pub struct MemorySystem {
    /// Three-tier memory hierarchy
    working_memory: Arc<RwLock<WorkingMemory>>,
    session_memory: Arc<RwLock<SessionMemory>>,
    long_term_memory: Arc<MemoryStorage>,

    /// Compression pipeline for consolidation
    compressor: CompressionPipeline,

    /// Semantic retrieval with graph associations
    retriever: RetrievalEngine,
}`}
      />

      <h2>Semantic Consolidation: Episodic to Semantic</h2>
      <p>
        Raw episodic memories ("user clicked button X at 3pm") aren't useful long-term.
        Semantic consolidation extracts patterns and converts them into facts:
      </p>

      <CodeBlock
        language="rust"
        filename="compression.rs"
        code={`/// Semantic consolidation: episodic → semantic transformation
/// Like what happens during human sleep
pub struct SemanticConsolidator {
    /// Minimum times a pattern must appear to become a fact
    min_support: usize,
    /// Minimum age before consolidation (days)
    min_age_days: u32,
}

pub enum FactType {
    Preference,    // "prefers dark mode"
    Capability,    // "can handle 10k req/sec"
    Relationship,  // "auth depends on redis"
    Procedure,     // "to deploy, run make release"
    Definition,    // "JWT is JSON Web Token"
    Pattern,       // "errors spike at 3am"
}

pub struct SemanticFact {
    pub fact: String,
    pub confidence: f32,
    pub support_count: usize,  // How many episodic memories support this
    pub source_ids: Vec<String>,
    pub fact_type: FactType,
}`}
      />
      <p>
        Example: if an agent encounters "API timeout when batch &gt; 100" five times,
        the consolidator extracts the fact: "Capability: batch size limited to 100"
        with confidence proportional to the support count.
      </p>

      <h2>Rich Context: Beyond Simple Key-Value</h2>
      <p>
        Context isn't a single thing. The system tracks multiple dimensions:
      </p>

      <CodeBlock
        language="rust"
        filename="context.rs"
        code={`pub struct RichContext {
    pub conversation: ConversationContext,  // Current dialogue state
    pub user: UserContext,                  // User preferences, history
    pub project: ProjectContext,            // Codebase, dependencies
    pub temporal: TemporalContext,          // Time of day, session duration
    pub semantic: SemanticContext,          // Active topics, entities
    pub code: CodeContext,                  // Files, functions in focus
    pub document: DocumentContext,          // Docs being referenced
    pub environment: EnvironmentContext,    // OS, shell, working dir

    pub decay_rate: f32,  // How fast this context loses relevance
    pub embeddings: Option<Vec<f32>>,  // Semantic vector for similarity
}`}
      />
      <p>
        This multi-dimensional context allows retrieval to be nuanced. Looking for
        memories about "authentication"? The system considers not just semantic
        similarity but also which project you're in, what files you're editing,
        and what you've been discussing recently.
      </p>

      <h2>Salience: What Gets Remembered</h2>
      <p>
        Not every piece of information deserves the same treatment. Salience scoring
        determines what gets prioritized during retrieval:
      </p>

      <CodeBlock
        language="rust"
        filename="constants.rs"
        code={`// Salience weights based on cognitive psychology research
pub const SALIENCE_RECENCY_WEIGHT: f32 = 0.3;   // Recent = more salient
pub const SALIENCE_FREQUENCY_WEIGHT: f32 = 0.25; // Frequently accessed = important
pub const SALIENCE_IMPORTANCE_WEIGHT: f32 = 0.25; // Explicit importance markers
pub const SALIENCE_SIMILARITY_WEIGHT: f32 = 0.2;  // Semantic relevance

// Matches Ebbinghaus forgetting curve decay rates
pub const BASE_HALF_LIFE_HOURS: f64 = 24.0;`}
      />

      <h2>Practical Application</h2>
      <p>
        The end result: an agent that remembers what worked, forgets what didn't,
        and builds stronger associations between related concepts over time.
      </p>

      <CodeBlock
        language="python"
        filename="example.py"
        code={`from shodh_memory import MemorySystem

memory = MemorySystem("./agent_memory")

# Record experiences
memory.record("User prefers dark mode", type="Context")
memory.record("API timeout when batch size > 100", type="Error")
memory.record("Switched to streaming for large requests", type="Decision")

# Retrieve relevant memories
results = memory.recall("API performance issues", limit=5)
# Returns: timeout error + streaming decision (high co-activation)`}
      />

      <h2>The Numbers</h2>
      <p>
        Some benchmarks from production use:
      </p>
      <ul>
        <li>50-80ms retrieval latency on 10K+ memories</li>
        <li>~90% token savings vs. stuffing full history into context</li>
        <li>6MB binary, runs on Raspberry Pi 4</li>
      </ul>
      <p>
        The architecture isn't novel - it's based on decades of cognitive science research.
        The contribution is packaging it into a practical system for AI agents.
      </p>
    </article>
  );
}

function EmbodiedAIMemoryPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        Humanoid robots and autonomous vehicles can't wait 200ms for a cloud round-trip.
        When a drone detects an obstacle at 15m/s, latency isn't a feature request - it's
        the difference between avoiding a collision and becoming one.
      </p>

      <h2>The Cloud Latency Problem</h2>
      <p>
        Most AI memory systems (Mem0, Zep, LangGraph Memory) are designed for chatbots.
        They assume:
      </p>
      <ul>
        <li>Stable internet connectivity</li>
        <li>Latency tolerance in the hundreds of milliseconds</li>
        <li>Centralized storage is acceptable</li>
      </ul>
      <p>
        These assumptions break down for embodied AI. A warehouse robot navigating at 2m/s
        needs sub-100ms decision cycles. A delivery drone operating in rural areas may have
        intermittent connectivity. A surgical robot cannot have its memory dependent on
        network availability.
      </p>

      <h2>Edge-Native Architecture</h2>
      <p>
        Edge-native means the memory system runs entirely on the device:
      </p>

      <CodeBlock
        language="python"
        filename="ros2_node.py"
        code={`from shodh_memory import MemorySystem
import rclpy
from rclpy.node import Node

class RobotMemoryNode(Node):
    def __init__(self):
        super().__init__('robot_memory')

        # Memory runs in-process - no network calls
        self.memory = MemorySystem("/robot/memory_db")

        self.obstacle_sub = self.create_subscription(
            LaserScan, '/scan', self.obstacle_callback, 10)

    def obstacle_callback(self, msg):
        # 5ms to record, 15ms to recall - no cloud dependency
        if min(msg.ranges) < 0.5:
            self.memory.record_obstacle(
                distance=min(msg.ranges),
                position=self.get_position()
            )

            # Check if we've seen similar obstacles before
            similar = self.memory.recall(
                f"obstacle at {self.get_zone()}", limit=3
            )
            if len(similar) > 2:
                self.get_logger().warn("Repeated obstacle zone")`}
      />

      <h2>Why RocksDB, Not SQLite</h2>
      <p>
        RocksDB is an LSM-tree storage engine optimized for write-heavy workloads.
        Robotics memory is write-heavy: constant sensor readings, state updates,
        decision logs. SQLite's B-tree structure would create write amplification.
      </p>
      <p>
        The tradeoff: RocksDB has higher read latency for point queries. But with
        semantic indexing (Vamana HNSW), reads go through the vector index anyway,
        not direct key lookups.
      </p>

      <h2>Memory Consolidation for Robots</h2>
      <p>
        Robots generate massive amounts of data. A LiDAR at 10Hz produces 600 scans
        per minute. You can't store everything. Consolidation compresses episodic
        memories (individual events) into semantic memories (patterns):
      </p>

      <CodeBlock
        language="rust"
        filename="compression.rs"
        code={`/// Consolidation: episodic → semantic transformation
/// Similar to what happens during human sleep
pub struct SemanticConsolidator {
    min_age_days: u32,
    similarity_threshold: f32,
}

impl SemanticConsolidator {
    pub fn consolidate(&self, memories: Vec<Memory>) -> Vec<SemanticFact> {
        // Group similar memories by semantic clustering
        let clusters = self.cluster_by_embedding(memories);

        clusters.into_iter().map(|cluster| {
            SemanticFact {
                // Extract common pattern from cluster
                content: self.extract_pattern(&cluster),
                support_count: cluster.len(),
                confidence: cluster.avg_importance(),
                fact_type: self.classify_fact(&cluster),
            }
        }).collect()
    }
}`}
      />

      <h2>Offline-First, Sync-Optional</h2>
      <p>
        The system is designed to work fully offline. If you want multi-robot
        synchronization, that's an optional layer:
      </p>
      <ul>
        <li>Each robot maintains its own RocksDB instance</li>
        <li>Sync happens opportunistically when connectivity allows</li>
        <li>Conflict resolution uses last-write-wins with vector clocks</li>
      </ul>
      <p>
        This matches how biological systems work: your brain doesn't need WiFi
        to form memories.
      </p>

      <h2>Deployment Footprint</h2>
      <table>
        <thead>
          <tr><th>Component</th><th>Size</th></tr>
        </thead>
        <tbody>
          <tr><td>Core binary</td><td>6MB</td></tr>
          <tr><td>MiniLM-L6-v2 model</td><td>22MB</td></tr>
          <tr><td>ONNX Runtime</td><td>50MB</td></tr>
          <tr><td>Total</td><td>~78MB</td></tr>
        </tbody>
      </table>
      <p>
        Runs on: Raspberry Pi 4, NVIDIA Jetson Nano, any x86_64 with 2GB RAM.
      </p>
    </article>
  );
}

function ROS2TutorialPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        This is a practical guide to adding persistent memory to ROS2 robots.
        We'll implement a memory node that records failures, tracks waypoints,
        and learns from operational experience.
      </p>

      <h2>Prerequisites</h2>
      <ul>
        <li>ROS2 Humble or later</li>
        <li>Python 3.8+</li>
        <li>shodh-memory: <code>pip install shodh-memory</code></li>
      </ul>

      <h2>Step 1: Create the Memory Node</h2>
      <p>
        We'll create a ROS2 node that wraps the memory system and exposes it
        through services:
      </p>

      <CodeBlock
        language="python"
        filename="memory_node.py"
        code={`#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from shodh_memory import MemorySystem, Position

from robot_interfaces.srv import RecordMemory, RecallMemory
from robot_interfaces.msg import RobotState

class MemoryNode(Node):
    def __init__(self):
        super().__init__('robot_memory')

        # Initialize memory system with persistent storage
        memory_path = self.declare_parameter('memory_path', '/robot/memory').value
        self.memory = MemorySystem(memory_path)
        self.get_logger().info(f'Memory initialized at {memory_path}')

        # Services for recording and recalling
        self.record_srv = self.create_service(
            RecordMemory, 'record_memory', self.record_callback)
        self.recall_srv = self.create_service(
            RecallMemory, 'recall_memory', self.recall_callback)

        # Subscribe to robot state for automatic failure detection
        self.state_sub = self.create_subscription(
            RobotState, '/robot/state', self.state_callback, 10)

        self.last_state = None

    def record_callback(self, request, response):
        memory_id = self.memory.record(
            content=request.content,
            type=request.memory_type or "Observation"
        )
        response.success = True
        response.memory_id = memory_id
        return response

    def recall_callback(self, request, response):
        results = self.memory.recall(request.query, limit=request.limit or 5)
        response.memories = [
            {"id": r["id"], "content": r["content"], "importance": r.get("importance", 0)}
            for r in results
        ]
        return response

    def state_callback(self, msg):
        if self.last_state is None:
            self.last_state = msg
            return

        # Detect state transitions that indicate failures
        if self.last_state.operational and not msg.operational:
            self.memory.record_failure(
                description=f"Robot went non-operational: {msg.error_code}",
                severity="high" if msg.emergency_stop else "medium",
                root_cause=msg.error_description
            )
            self.get_logger().warn(f'Recorded failure: {msg.error_code}')

        self.last_state = msg

def main():
    rclpy.init()
    node = MemoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`}
      />

      <h2>Step 2: Define Service Interfaces</h2>
      <p>
        Create the service definitions in your robot_interfaces package:
      </p>

      <CodeBlock
        language="text"
        filename="srv/RecordMemory.srv"
        code={`string content
string memory_type
---
bool success
string memory_id`}
      />

      <CodeBlock
        language="text"
        filename="srv/RecallMemory.srv"
        code={`string query
int32 limit
---
string[] memories`}
      />

      <h2>Step 3: Waypoint Memory</h2>
      <p>
        Record visited waypoints with success/failure status:
      </p>

      <CodeBlock
        language="python"
        filename="waypoint_tracker.py"
        code={`from shodh_memory import MemorySystem, Position

class WaypointTracker:
    def __init__(self, memory: MemorySystem):
        self.memory = memory

    def record_waypoint(self, name: str, pos: tuple, status: str, notes: str = ""):
        position = Position(x=pos[0], y=pos[1], z=pos[2] if len(pos) > 2 else 0.0)

        self.memory.record_waypoint(
            name=name,
            position=position,
            status=status,  # "reached", "failed", "skipped"
            notes=notes
        )

    def get_problem_waypoints(self) -> list:
        """Find waypoints that frequently fail"""
        results = self.memory.recall("waypoint failed", limit=20)

        # Count failures per waypoint name
        failure_counts = {}
        for r in results:
            # Extract waypoint name from memory content
            if "waypoint" in r["content"].lower():
                name = self.extract_waypoint_name(r["content"])
                failure_counts[name] = failure_counts.get(name, 0) + 1

        # Return waypoints with 2+ failures
        return [name for name, count in failure_counts.items() if count >= 2]

    def extract_waypoint_name(self, content: str) -> str:
        # Simple extraction - customize for your naming scheme
        import re
        match = re.search(r'waypoint[\\s:]+([\\w-]+)', content, re.I)
        return match.group(1) if match else "unknown"`}
      />

      <h2>Step 4: Learning from Experience</h2>
      <p>
        The Hebbian learning mechanism means frequently co-accessed memories
        strengthen their connections. When you recall "navigation failure" and
        a specific sensor reading appears in multiple results, that association
        gets stronger.
      </p>

      <CodeBlock
        language="python"
        filename="diagnostic_helper.py"
        code={`def diagnose_failure(memory: MemorySystem, symptoms: str) -> list:
    """
    Given current symptoms, find relevant past failures and their resolutions.
    Hebbian learning ensures frequently successful patterns rank higher.
    """
    # Search for similar past issues
    similar_failures = memory.recall(
        f"failure {symptoms}",
        limit=10
    )

    # For each failure, find associated resolutions
    resolutions = []
    for failure in similar_failures:
        # Search for decisions made after this failure
        followup = memory.recall(
            f"decision after {failure['id'][:8]}",
            limit=3
        )
        if followup:
            resolutions.append({
                "failure": failure["content"],
                "resolution": followup[0]["content"],
                "confidence": failure.get("importance", 0) * followup[0].get("importance", 0)
            })

    return sorted(resolutions, key=lambda x: x["confidence"], reverse=True)`}
      />

      <h2>Step 5: Launch Configuration</h2>
      <p>
        Add the memory node to your robot's launch file:
      </p>

      <CodeBlock
        language="python"
        filename="robot_launch.py"
        code={`from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_memory',
            executable='memory_node',
            name='robot_memory',
            parameters=[{
                'memory_path': '/robot/persistent_memory'
            }],
            output='screen'
        ),
        # ... other nodes
    ])`}
      />

      <h2>Memory Maintenance</h2>
      <p>
        The system runs background consolidation automatically. For long-running
        robots, old memories compress into semantic facts:
      </p>
      <ul>
        <li>Individual "navigation failed at waypoint X" events consolidate into "waypoint X has reliability issues"</li>
        <li>Repeated sensor readings consolidate into "LiDAR unreliable in dusty conditions"</li>
        <li>Activation decay prunes memories that haven't been accessed in weeks</li>
      </ul>
      <p>
        The result: a robot that gets more reliable over time because it remembers
        what went wrong and what fixed it.
      </p>
    </article>
  );
}

// New Blog Posts - Shodh Memory focused

function PythonSDKTutorialPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        The Shodh Memory Python SDK gives your applications persistent, semantic memory
        in just a few lines of code. This tutorial covers everything from installation
        to advanced patterns.
      </p>

      <h2>Installation</h2>
      <p>Install from PyPI:</p>

      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`pip install shodh-memory`}
      />

      <p>That's it. No external services, no API keys required for local use.</p>

      <h2>Quick Start</h2>
      <p>Here's the minimal example to get started:</p>

      <CodeBlock
        language="python"
        filename="quickstart.py"
        code={`from shodh_memory import Memory

# Initialize (creates local storage in ./shodh_data by default)
m = Memory()

# Store a memory
m.remember("User prefers dark mode in all applications",
           memory_type="Decision",
           tags=["preferences", "ui"])

# Recall memories by semantic search
results = m.recall("What are the user's UI preferences?")
for r in results:
    print(f"[{r['experience_type']}] {r['content']} (importance: {r.get('importance', 0):.2f})")`}
      />

      <h2>Memory Types</h2>
      <p>
        Shodh Memory supports different memory types, each with different importance
        weights during retrieval:
      </p>

      <table>
        <thead>
          <tr><th>Type</th><th>Weight Boost</th><th>Use Case</th></tr>
        </thead>
        <tbody>
          <tr><td><code>Decision</code></td><td>+30%</td><td>Choices, preferences, architectural decisions</td></tr>
          <tr><td><code>Learning</code></td><td>+25%</td><td>New knowledge, discoveries</td></tr>
          <tr><td><code>Error</code></td><td>+25%</td><td>Mistakes to avoid, bugs encountered</td></tr>
          <tr><td><code>Pattern</code></td><td>+20%</td><td>Recurring behaviors, habits</td></tr>
          <tr><td><code>Task</code></td><td>+15%</td><td>Work items, todos</td></tr>
          <tr><td><code>Context</code></td><td>+10%</td><td>General information, session context</td></tr>
          <tr><td><code>Observation</code></td><td>+0%</td><td>Default type for general notes</td></tr>
        </tbody>
      </table>

      <h2>Retrieval Modes</h2>
      <p>The SDK supports three retrieval modes:</p>

      <CodeBlock
        language="python"
        filename="retrieval_modes.py"
        code={`# Semantic mode: Pure vector similarity search
results = m.recall("authentication", mode="semantic", limit=5)

# Associative mode: Graph-based traversal
# Follows learned connections between memories
results = m.recall("authentication", mode="associative", limit=5)

# Hybrid mode (default): Combines both
# Uses density-dependent weighting
results = m.recall("authentication", mode="hybrid", limit=5)`}
      />

      <h2>Context Summary</h2>
      <p>
        Get a structured summary of memories - perfect for bootstrapping LLM context
        at the start of a session:
      </p>

      <CodeBlock
        language="python"
        filename="context_summary.py"
        code={`summary = m.context_summary(max_items=5)

print("Decisions:", summary["decisions"])
print("Learnings:", summary["learnings"])
print("Errors to avoid:", summary.get("errors", []))
print("Patterns:", summary.get("patterns", []))

# Use in LLM prompt
system_prompt = f"""You are helping a user. Here's what you know:

Decisions: {summary["decisions"]}
Learnings: {summary["learnings"]}
Errors to avoid: {summary.get("errors", [])}
"""`}
      />

      <h2>Tag-Based Retrieval</h2>
      <p>Search memories by tags for precise filtering:</p>

      <CodeBlock
        language="python"
        filename="tags.py"
        code={`# Store with tags
m.remember("PostgreSQL connection pool size should be 20",
           memory_type="Learning",
           tags=["database", "postgres", "performance"])

# Retrieve by tags
db_memories = m.recall_by_tags(["database", "postgres"])

# Delete by tags
m.forget_by_tags(["temporary", "scratch"])`}
      />

      <h2>Date-Based Operations</h2>
      <p>Query and manage memories by date range:</p>

      <CodeBlock
        language="python"
        filename="date_operations.py"
        code={`from datetime import datetime, timedelta

# Get memories from the last 7 days
week_ago = (datetime.now() - timedelta(days=7)).isoformat()
now = datetime.now().isoformat()

recent = m.recall_by_date(start=week_ago, end=now)

# Clean up old memories
month_ago = (datetime.now() - timedelta(days=30)).isoformat()
m.forget_by_date(start="2024-01-01", end=month_ago)`}
      />

      <h2>Memory Statistics</h2>
      <p>Monitor your memory system:</p>

      <CodeBlock
        language="python"
        filename="stats.py"
        code={`stats = m.memory_stats()

print(f"Total memories: {stats.total_memories}")
print(f"Graph edges: {stats.graph_edges}")
print(f"Storage size: {stats.storage_size_bytes / 1024:.1f} KB")`}
      />

      <h2>Integration with LangChain</h2>
      <p>Use Shodh Memory as a LangChain memory backend:</p>

      <CodeBlock
        language="python"
        filename="langchain_integration.py"
        code={`from shodh_memory import Memory
from langchain.memory import ConversationBufferMemory

class ShodhLangChainMemory(ConversationBufferMemory):
    def __init__(self, shodh_memory: Memory):
        super().__init__()
        self.shodh = shodh_memory

    def save_context(self, inputs, outputs):
        # Store in Shodh for persistence
        self.shodh.remember(
            f"User: {inputs['input']}\\nAssistant: {outputs['output']}",
            memory_type="Conversation"
        )
        super().save_context(inputs, outputs)

    def load_memory_variables(self, inputs):
        # Retrieve relevant context from Shodh
        if 'input' in inputs:
            relevant = self.shodh.recall(inputs['input'], limit=3)
            # Add to context...
        return super().load_memory_variables(inputs)`}
      />

      <h2>Custom Storage Path</h2>
      <p>Specify where memories are stored:</p>

      <CodeBlock
        language="python"
        filename="custom_path.py"
        code={`# Project-specific memory
project_memory = Memory(storage_path="./my_project/.shodh")

# User-specific memory
user_memory = Memory(storage_path="~/.shodh/personal")

# Temporary memory (will be deleted)
import tempfile
temp_memory = Memory(storage_path=tempfile.mkdtemp())`}
      />

      <h2>Best Practices</h2>
      <ul>
        <li><strong>Be specific with types</strong> — Use Decision for choices, Error for mistakes. This improves retrieval relevance.</li>
        <li><strong>Tag liberally</strong> — Tags enable precise filtering. Use project names, topics, technologies.</li>
        <li><strong>Use context_summary for LLMs</strong> — At session start, bootstrap with recent decisions and learnings.</li>
        <li><strong>Clean up periodically</strong> — Use forget_by_date to remove old, irrelevant memories.</li>
        <li><strong>One Memory instance per user/project</strong> — Don't share instances across different contexts.</li>
      </ul>

      <h2>Next Steps</h2>
      <ul>
        <li><a href="https://github.com/varun29ankuS/shodh-memory">GitHub Repository</a> — Full source and documentation</li>
        <li><a href="https://pypi.org/project/shodh-memory/">PyPI Package</a> — Version history and metadata</li>
        <li><a href="/blog/claude-code-mcp-server-setup-guide">MCP Setup Guide</a> — Use with Claude Code</li>
      </ul>
    </article>
  );
}

function HebbianLearningPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        Shodh Memory isn't just a key-value store with vector search. It's built on
        cognitive science principles—the same mechanisms that make human memory work.
        This post explains the architecture.
      </p>

      <h2>The Problem with Naive Memory</h2>
      <p>
        Most AI memory systems are glorified databases: store embeddings, run similarity
        search, return top-k results. This misses fundamental properties of biological memory:
      </p>
      <ul>
        <li>Memories strengthen with use and weaken without it</li>
        <li>Related memories activate each other (spreading activation)</li>
        <li>Raw experiences consolidate into abstract knowledge over time</li>
        <li>Not all information is equally important</li>
      </ul>

      <h2>Hebbian Learning: "Fire Together, Wire Together"</h2>
      <p>
        Donald Hebb's 1949 theory states that when neurons repeatedly fire together,
        the connection between them strengthens. In Shodh Memory, when two memories
        are retrieved in the same context, we strengthen their association:
      </p>

      <CodeBlock
        language="rust"
        filename="hebbian.rs"
        code={`/// When memories A and B are retrieved together, strengthen their edge
pub fn strengthen_association(&mut self, memory_a: &str, memory_b: &str) {
    let edge = self.get_or_create_edge(memory_a, memory_b);

    // Hebbian strengthening with diminishing returns
    let delta = LEARNING_RATE * (1.0 - edge.strength);
    edge.strength = (edge.strength + delta).min(1.0);

    edge.activation_count += 1;
    edge.last_activated = Utc::now();

    // Check for Long-Term Potentiation
    if edge.activation_count >= LTP_THRESHOLD && !edge.potentiated {
        edge.potentiated = true;  // Becomes permanent
    }
}`}
      />

      <p>
        The key insight: frequently co-accessed memories form strong associations.
        When you retrieve "authentication" and "JWT tokens" together multiple times,
        future queries for "authentication" will naturally surface JWT-related memories.
      </p>

      <h2>Long-Term Potentiation (LTP)</h2>
      <p>
        In neuroscience, LTP is the process where synapses become permanently strengthened
        after sufficient repetition. We implement this:
      </p>

      <CodeBlock
        language="rust"
        filename="ltp.rs"
        code={`const LTP_THRESHOLD: u32 = 5;  // Activations needed for potentiation
const LTP_DECAY_FACTOR: f32 = 0.1;  // Potentiated edges decay 10x slower

impl Edge {
    pub fn decay(&mut self) -> bool {
        let half_life = if self.potentiated {
            BASE_HALF_LIFE * 10.0  // Much slower decay
        } else {
            BASE_HALF_LIFE * (1.0 + self.strength as f64)
        };

        let elapsed_days = self.days_since_access();
        self.strength *= (-0.693 / half_life * elapsed_days).exp() as f32;

        self.strength < MIN_STRENGTH  // Return true if should prune
    }
}`}
      />

      <p>
        After 5 co-activations, an association becomes "potentiated"—it decays 10x slower.
        This means core knowledge persists while ephemeral associations fade.
      </p>

      <h2>Spreading Activation</h2>
      <p>
        When you recall a memory, activation spreads to connected memories. This models
        how human memory retrieval works—thinking of "Paris" activates "France", "Eiffel Tower",
        "croissants":
      </p>

      <CodeBlock
        language="rust"
        filename="spreading_activation.rs"
        code={`pub fn spreading_activation(
    &self,
    seed_memories: &[MemoryId],
    depth: usize,
    decay_factor: f32,
) -> HashMap<MemoryId, f32> {
    let mut activations: HashMap<MemoryId, f32> = HashMap::new();
    let mut frontier: VecDeque<(MemoryId, f32, usize)> = VecDeque::new();

    // Initialize with seed memories at full activation
    for id in seed_memories {
        activations.insert(id.clone(), 1.0);
        frontier.push_back((id.clone(), 1.0, 0));
    }

    while let Some((current, activation, current_depth)) = frontier.pop_front() {
        if current_depth >= depth {
            continue;
        }

        // Spread to neighbors
        for (neighbor, edge) in self.get_neighbors(&current) {
            let spread_activation = activation * edge.strength * decay_factor;

            if spread_activation > MIN_ACTIVATION {
                let existing = activations.get(&neighbor).unwrap_or(&0.0);
                if spread_activation > *existing {
                    activations.insert(neighbor.clone(), spread_activation);
                    frontier.push_back((neighbor, spread_activation, current_depth + 1));
                }
            }
        }
    }

    activations
}`}
      />

      <h2>Memory Consolidation</h2>
      <p>
        Raw episodic memories ("API timed out at 3pm") aren't useful long-term.
        Consolidation extracts semantic facts ("API has 30-second timeout"):
      </p>

      <CodeBlock
        language="rust"
        filename="consolidation.rs"
        code={`pub struct ConsolidationEngine {
    /// Minimum times a pattern must appear to become a fact
    min_support: usize,
    /// Minimum age before consolidation (days)
    min_age_days: u32,
}

impl ConsolidationEngine {
    pub fn consolidate(&self, episodic_memories: &[Memory]) -> Vec<SemanticFact> {
        // Cluster similar memories by embedding similarity
        let clusters = self.cluster_by_embedding(episodic_memories);

        clusters.into_iter()
            .filter(|c| c.len() >= self.min_support)
            .map(|cluster| {
                SemanticFact {
                    content: self.extract_common_pattern(&cluster),
                    confidence: cluster.len() as f32 / episodic_memories.len() as f32,
                    source_count: cluster.len(),
                    fact_type: self.classify_fact_type(&cluster),
                }
            })
            .collect()
    }
}`}
      />

      <h2>Three-Tier Architecture</h2>
      <p>Shodh Memory organizes storage in three tiers:</p>

      <table>
        <thead>
          <tr><th>Tier</th><th>Purpose</th><th>Lifetime</th></tr>
        </thead>
        <tbody>
          <tr><td><strong>Working Memory</strong></td><td>Current context, active associations</td><td>Session</td></tr>
          <tr><td><strong>Episodic Memory</strong></td><td>Specific events and experiences</td><td>Days to weeks</td></tr>
          <tr><td><strong>Semantic Memory</strong></td><td>Consolidated facts and knowledge</td><td>Permanent (potentiated)</td></tr>
        </tbody>
      </table>

      <h2>Salience Scoring</h2>
      <p>
        Not all memories are equally important. Salience combines multiple signals:
      </p>

      <CodeBlock
        language="rust"
        filename="salience.rs"
        code={`pub fn compute_salience(memory: &Memory, query_embedding: &[f32]) -> f32 {
    let recency = recency_score(memory.last_accessed);      // 30%
    let frequency = frequency_score(memory.access_count);    // 25%
    let importance = importance_score(memory.memory_type);   // 25%
    let similarity = cosine_similarity(&memory.embedding, query_embedding); // 20%

    0.30 * recency +
    0.25 * frequency +
    0.25 * importance +
    0.20 * similarity
}

fn importance_score(memory_type: MemoryType) -> f32 {
    match memory_type {
        MemoryType::Decision => 1.3,
        MemoryType::Error => 1.25,
        MemoryType::Learning => 1.25,
        MemoryType::Pattern => 1.2,
        MemoryType::Task => 1.15,
        MemoryType::Context => 1.1,
        _ => 1.0,
    }
}`}
      />

      <h2>The Result</h2>
      <p>
        These mechanisms combine to create memory that:
      </p>
      <ul>
        <li><strong>Gets smarter with use</strong> — Frequently accessed knowledge surfaces faster</li>
        <li><strong>Forgets appropriately</strong> — Unused information fades, reducing noise</li>
        <li><strong>Connects ideas</strong> — Related concepts strengthen each other</li>
        <li><strong>Prioritizes importance</strong> — Decisions and errors rank higher than casual observations</li>
      </ul>

      <p>
        The implementation is ~6MB of Rust, runs entirely locally, and achieves sub-10ms
        retrieval on tens of thousands of memories. It's cognitive science, not magic.
      </p>
    </article>
  );
}

function AIMemoryUseCasesPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        "Why does my AI assistant keep forgetting what I told it yesterday?"
        This is the most common frustration with AI tools. Here are 5 concrete ways
        persistent memory transforms your development workflow.
      </p>

      <h2>1. Project Context Retention</h2>
      <p>
        Every new coding session starts with: "This project uses Next.js 14 with App Router,
        TypeScript strict mode, and Tailwind CSS..." With persistent memory, you tell it once:
      </p>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`> Explore this codebase and remember the key architectural decisions

[Claude explores and stores:]
- "This project uses Next.js 14 with App Router"
- "Database is PostgreSQL via Prisma ORM"
- "Auth is NextAuth with GitHub OAuth"
- "CSS is Tailwind with custom design tokens"

# Next day, new session:
> Add a new API endpoint for user preferences

[Claude already knows the tech stack, no re-explanation needed]`}
      />

      <p>
        The memory persists across sessions. Claude remembers your project structure,
        naming conventions, and architectural decisions.
      </p>

      <h2>2. Preference Learning</h2>
      <p>
        You have opinions: tabs vs spaces, functional vs class components, error handling
        patterns. Without memory, you repeat these preferences constantly:
      </p>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`> Remember my coding preferences:
> - Always use functional components with hooks
> - TypeScript strict mode, no 'any' types
> - Error handling: try-catch with custom error classes
> - Tests: Vitest with React Testing Library
> - Comments: JSDoc for public APIs only

[Stored as Decision type memories]

# Future sessions automatically apply these preferences
> Create a new user profile component

[Claude generates functional component, strict TypeScript,
with Vitest tests—no reminders needed]`}
      />

      <h2>3. Error Tracking & Prevention</h2>
      <p>
        "We fixed this bug before, but nobody remembers how." Memory captures errors
        and their solutions:
      </p>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`> The tests are failing with "Cannot find module '@/components/Button'"

[After investigation:]
> Remember: Path aliases require tsconfig.json paths to be mirrored
> in vitest.config.ts under resolve.alias

[Stored as Error type memory]

# Weeks later, similar issue:
> Why can't Vitest find my aliased imports?

[Claude recalls the previous error and solution immediately]`}
      />

      <p>
        Error memories get a 25% importance boost, so they surface quickly when
        relevant issues arise.
      </p>

      <h2>4. Knowledge Accumulation</h2>
      <p>
        Every debugging session teaches something. Without memory, that knowledge
        evaporates. With memory, it compounds:
      </p>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`# Day 1:
> Remember: React Query's staleTime should be set higher for
> data that doesn't change often. We use 5 minutes for user profiles.

# Day 5:
> Remember: The Stripe webhook needs the raw body, so we disable
> body parsing for that route only.

# Day 12:
> Remember: PostgreSQL connection pool exhaustion happens when
> we don't close connections in serverless functions. Use
> connection poolers like PgBouncer.

# Day 30:
> Give me a summary of learnings about this project

[Claude retrieves all Learning type memories, providing
a knowledge base that grew organically over time]`}
      />

      <h2>5. Cross-Session Continuity</h2>
      <p>
        Large tasks span multiple sessions. Memory maintains continuity:
      </p>

      <CodeBlock
        language="text"
        filename="Claude Code Session"
        code={`# Session 1 (Monday):
> We're refactoring the auth system from session-based to JWT.
> Today we updated the login endpoint.

[Claude stores task context]

# Session 2 (Tuesday):
> What was I working on with the auth refactor?

[Claude recalls:]
"You were refactoring auth from session to JWT.
Login endpoint is updated. Remaining: logout, refresh tokens,
middleware updates."

# Session continues without re-explaining the whole context`}
      />

      <h2>Setting It Up</h2>
      <p>Getting these benefits takes 2 minutes:</p>

      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`# 1. Install the memory server
pip install shodh-memory

# 2. Add to Claude Code
claude mcp add @shodh/memory-mcp

# 3. Start using it
claude
> Remember that this project uses PostgreSQL with Prisma`}
      />

      <h2>The Compound Effect</h2>
      <p>
        Each of these use cases is valuable alone. Together, they compound:
      </p>
      <ul>
        <li>Project context eliminates repetitive explanations</li>
        <li>Preferences ensure consistent code style</li>
        <li>Error tracking prevents repeated debugging</li>
        <li>Knowledge accumulation builds institutional memory</li>
        <li>Cross-session continuity maintains flow</li>
      </ul>
      <p>
        After a month of use, your AI assistant knows your codebase, your preferences,
        your past mistakes, and your accumulated learnings. It's like having a colleague
        who actually remembers everything.
      </p>

      <h2>Getting Started</h2>
      <ul>
        <li><a href="/docs">Documentation</a> — Quick start guide</li>
        <li><a href="/demo">Try in Colab</a> — Interactive notebook</li>
        <li><a href="https://github.com/varun29ankuS/shodh-memory">GitHub</a> — Source and issues</li>
      </ul>
    </article>
  );
}

function AIMemoryComparisonPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        Every AI agent needs memory. The question is: which memory system? This comparison
        covers the three leading solutions—Mem0, Zep, and Shodh—with honest assessments
        of when each shines and where each falls short.
      </p>

      <div className="not-prose bg-slate-100 dark:bg-slate-800 rounded-lg p-6 my-8">
        <h3 className="text-lg font-semibold text-slate-900 dark:text-white mb-4">TL;DR Summary</h3>
        <ul className="space-y-2 text-slate-700 dark:text-slate-300">
          <li><strong>Mem0:</strong> Best for cloud-first teams wanting managed infrastructure</li>
          <li><strong>Zep:</strong> Best for applications needing temporal reasoning and chat history</li>
          <li><strong>Shodh:</strong> Best for edge devices, offline operation, and privacy-sensitive deployments</li>
        </ul>
      </div>

      <h2>The Contenders</h2>

      <h3>Mem0 — The Well-Funded Cloud Player</h3>
      <p>
        Mem0 raised <strong>$24M from AWS</strong> in late 2024, making it the most well-funded
        AI memory startup. They position as "the memory layer for AI applications" with
        deep AWS integration and a managed cloud service.
      </p>
      <ul>
        <li><strong>GitHub:</strong> 41K+ stars</li>
        <li><strong>Model:</strong> Cloud-first SaaS with self-hosted option</li>
        <li><strong>Integrations:</strong> LangChain, LlamaIndex, CrewAI</li>
        <li><strong>Unique feature:</strong> AWS Bedrock native integration</li>
      </ul>

      <h3>Zep — The Temporal Memory Specialist</h3>
      <p>
        Zep focuses on <strong>temporal knowledge graphs</strong>—understanding when things happened
        and how relationships change over time. They've raised approximately $500K and have
        strong LangChain integration.
      </p>
      <ul>
        <li><strong>GitHub:</strong> ~20K stars</li>
        <li><strong>Model:</strong> Cloud service + open-source</li>
        <li><strong>Integrations:</strong> LangChain (official partner)</li>
        <li><strong>Unique feature:</strong> Temporal reasoning, fact versioning</li>
      </ul>

      <h3>Shodh — The Edge-Native Option</h3>
      <p>
        Shodh Memory is built for <strong>edge devices and offline operation</strong>. Single binary,
        no external dependencies, runs on Raspberry Pi. Uses cognitive science principles
        (Hebbian learning) for memory consolidation.
      </p>
      <ul>
        <li><strong>GitHub:</strong> Open source</li>
        <li><strong>Model:</strong> Fully local, single binary</li>
        <li><strong>Integrations:</strong> MCP (Claude Code), Python SDK</li>
        <li><strong>Unique feature:</strong> Offline-first, sub-millisecond latency</li>
      </ul>

      <h2>Architecture Comparison</h2>

      <table>
        <thead>
          <tr>
            <th>Aspect</th>
            <th>Mem0</th>
            <th>Zep</th>
            <th>Shodh</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><strong>Deployment</strong></td>
            <td>Cloud SaaS / Self-hosted</td>
            <td>Cloud SaaS / Self-hosted</td>
            <td>Local binary only</td>
          </tr>
          <tr>
            <td><strong>Storage Backend</strong></td>
            <td>PostgreSQL + Qdrant</td>
            <td>PostgreSQL + pgvector</td>
            <td>RocksDB (embedded)</td>
          </tr>
          <tr>
            <td><strong>Vector Search</strong></td>
            <td>Qdrant</td>
            <td>pgvector</td>
            <td>Vamana HNSW (custom)</td>
          </tr>
          <tr>
            <td><strong>Embedding Model</strong></td>
            <td>OpenAI / Custom</td>
            <td>OpenAI / Custom</td>
            <td>MiniLM-L6-v2 (local ONNX)</td>
          </tr>
          <tr>
            <td><strong>Graph Support</strong></td>
            <td>Entity extraction</td>
            <td>Temporal knowledge graph</td>
            <td>Hebbian associative graph</td>
          </tr>
          <tr>
            <td><strong>Binary Size</strong></td>
            <td>Docker container</td>
            <td>Docker container</td>
            <td>~15MB standalone</td>
          </tr>
        </tbody>
      </table>

      <h2>Performance Benchmarks</h2>
      <p>
        We benchmarked typical operations. Note: cloud services have network latency;
        Shodh runs entirely local.
      </p>

      <table>
        <thead>
          <tr>
            <th>Operation</th>
            <th>Mem0 (Cloud)</th>
            <th>Zep (Cloud)</th>
            <th>Shodh (Local)</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><strong>Memory Store</strong></td>
            <td>~200-400ms</td>
            <td>~150-300ms</td>
            <td>~5-15ms</td>
          </tr>
          <tr>
            <td><strong>Semantic Recall</strong></td>
            <td>~300-500ms</td>
            <td>~200-400ms</td>
            <td>~10-50ms</td>
          </tr>
          <tr>
            <td><strong>Entity Lookup</strong></td>
            <td>~100-200ms</td>
            <td>~100-200ms</td>
            <td>~0.7ms</td>
          </tr>
          <tr>
            <td><strong>Offline Operation</strong></td>
            <td>No</td>
            <td>No</td>
            <td>Yes</td>
          </tr>
        </tbody>
      </table>

      <p className="text-sm text-slate-600 dark:text-slate-400">
        * Cloud latencies include network round-trip. Self-hosted deployments would be faster
        but require infrastructure management.
      </p>

      <h2>Feature Comparison</h2>

      <h3>Memory Types & Organization</h3>
      <table>
        <thead>
          <tr>
            <th>Feature</th>
            <th>Mem0</th>
            <th>Zep</th>
            <th>Shodh</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Semantic search</td>
            <td>✅</td>
            <td>✅</td>
            <td>✅</td>
          </tr>
          <tr>
            <td>Memory types/categories</td>
            <td>✅</td>
            <td>✅</td>
            <td>✅ (13 types)</td>
          </tr>
          <tr>
            <td>Tagging</td>
            <td>✅</td>
            <td>✅</td>
            <td>✅</td>
          </tr>
          <tr>
            <td>Temporal queries</td>
            <td>Basic</td>
            <td>✅ Advanced</td>
            <td>✅</td>
          </tr>
          <tr>
            <td>Fact versioning</td>
            <td>❌</td>
            <td>✅</td>
            <td>❌</td>
          </tr>
          <tr>
            <td>Memory decay</td>
            <td>❌</td>
            <td>❌</td>
            <td>✅ (Ebbinghaus curve)</td>
          </tr>
          <tr>
            <td>Hebbian learning</td>
            <td>❌</td>
            <td>❌</td>
            <td>✅</td>
          </tr>
          <tr>
            <td>Memory consolidation</td>
            <td>❌</td>
            <td>❌</td>
            <td>✅</td>
          </tr>
        </tbody>
      </table>

      <h3>Integration & Ecosystem</h3>
      <table>
        <thead>
          <tr>
            <th>Integration</th>
            <th>Mem0</th>
            <th>Zep</th>
            <th>Shodh</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>LangChain</td>
            <td>✅</td>
            <td>✅ Official</td>
            <td>🔜 Coming</td>
          </tr>
          <tr>
            <td>LlamaIndex</td>
            <td>✅</td>
            <td>✅</td>
            <td>🔜 Coming</td>
          </tr>
          <tr>
            <td>Claude Code (MCP)</td>
            <td>❌</td>
            <td>❌</td>
            <td>✅</td>
          </tr>
          <tr>
            <td>Python SDK</td>
            <td>✅</td>
            <td>✅</td>
            <td>✅</td>
          </tr>
          <tr>
            <td>REST API</td>
            <td>✅</td>
            <td>✅</td>
            <td>✅</td>
          </tr>
          <tr>
            <td>ROS2 (Robotics)</td>
            <td>❌</td>
            <td>❌</td>
            <td>✅</td>
          </tr>
        </tbody>
      </table>

      <h2>Pricing Comparison</h2>

      <table>
        <thead>
          <tr>
            <th>Tier</th>
            <th>Mem0</th>
            <th>Zep</th>
            <th>Shodh</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><strong>Free</strong></td>
            <td>1K memories</td>
            <td>Limited</td>
            <td>Unlimited</td>
          </tr>
          <tr>
            <td><strong>Pro</strong></td>
            <td>$99/mo</td>
            <td>$20/mo</td>
            <td>Free (open source)</td>
          </tr>
          <tr>
            <td><strong>Enterprise</strong></td>
            <td>Custom</td>
            <td>Custom</td>
            <td>Free (open source)</td>
          </tr>
          <tr>
            <td><strong>Self-hosted</strong></td>
            <td>Available</td>
            <td>Available</td>
            <td>Only option</td>
          </tr>
        </tbody>
      </table>

      <h2>When to Use Each</h2>

      <h3>Choose Mem0 When:</h3>
      <ul>
        <li>You're building on AWS and want native integration</li>
        <li>You prefer managed infrastructure over self-hosting</li>
        <li>You need enterprise support and SLAs</li>
        <li>Your team is already using LangChain/LlamaIndex</li>
        <li>Budget isn't the primary constraint</li>
      </ul>

      <h3>Choose Zep When:</h3>
      <ul>
        <li>Temporal reasoning is critical (e.g., "What did the user say last week?")</li>
        <li>You need fact versioning (tracking how beliefs change over time)</li>
        <li>Chat history management is the primary use case</li>
        <li>You're deeply integrated with LangChain</li>
        <li>You need conversation summarization</li>
      </ul>

      <h3>Choose Shodh When:</h3>
      <ul>
        <li>Edge deployment (robots, drones, IoT devices)</li>
        <li>Offline operation is required</li>
        <li>Sub-millisecond latency matters</li>
        <li>Privacy/data sovereignty—data never leaves the device</li>
        <li>Minimal dependencies—single binary deployment</li>
        <li>Cognitive features (Hebbian learning, memory decay, consolidation)</li>
        <li>Cost sensitivity—fully open source, no usage fees</li>
      </ul>

      <h2>The Honest Trade-offs</h2>

      <h3>Mem0's Trade-offs</h3>
      <ul>
        <li><strong>Pro:</strong> Best-funded, most integrations, enterprise support</li>
        <li><strong>Con:</strong> Cloud dependency, higher cost, network latency</li>
        <li><strong>Con:</strong> AWS-centric (if you're multi-cloud)</li>
      </ul>

      <h3>Zep's Trade-offs</h3>
      <ul>
        <li><strong>Pro:</strong> Best temporal reasoning, official LangChain partner</li>
        <li><strong>Con:</strong> Smaller team, less funding than Mem0</li>
        <li><strong>Con:</strong> Still requires cloud/server infrastructure</li>
      </ul>

      <h3>Shodh's Trade-offs</h3>
      <ul>
        <li><strong>Pro:</strong> Only true edge/offline solution, no dependencies</li>
        <li><strong>Pro:</strong> Unique cognitive features (Hebbian, consolidation)</li>
        <li><strong>Con:</strong> Smaller ecosystem (no LangChain integration yet)</li>
        <li><strong>Con:</strong> No managed cloud option—you run it yourself</li>
        <li><strong>Con:</strong> Newer project, smaller community</li>
      </ul>

      <h2>Code Comparison</h2>

      <h3>Mem0</h3>
      <CodeBlock
        language="python"
        filename="mem0_example.py"
        code={`from mem0 import Memory

m = Memory()
m.add("User prefers dark mode", user_id="alice")

results = m.search("UI preferences", user_id="alice")
for r in results:
    print(r["memory"])`}
      />

      <h3>Zep</h3>
      <CodeBlock
        language="python"
        filename="zep_example.py"
        code={`from zep_cloud.client import Zep

client = Zep(api_key="your-key")

# Add memory
client.memory.add(
    session_id="session-123",
    messages=[{"role": "user", "content": "I prefer dark mode"}]
)

# Search
results = client.memory.search(
    session_id="session-123",
    text="UI preferences"
)`}
      />

      <h3>Shodh</h3>
      <CodeBlock
        language="python"
        filename="shodh_example.py"
        code={`from shodh_memory import Memory

m = Memory()  # Local storage, no API key needed
m.remember("User prefers dark mode",
           memory_type="Decision",
           tags=["preferences", "ui"])

results = m.recall("UI preferences")
for r in results:
    print(f"[{r['experience_type']}] {r['content']}")`}
      />

      <h2>Migration Paths</h2>
      <p>
        If you're switching between systems, here's what to know:
      </p>

      <h3>Mem0 → Shodh</h3>
      <p>
        Export memories via Mem0 API, import using Shodh's batch_remember endpoint.
        You'll lose Mem0-specific metadata but retain core content and timestamps.
      </p>

      <h3>Zep → Shodh</h3>
      <p>
        Export conversation history and facts. Zep's temporal metadata won't transfer
        directly—you'll need to flatten versioned facts into current state.
      </p>

      <h3>Shodh → Cloud (Mem0/Zep)</h3>
      <p>
        Export from Shodh's REST API or directly from RocksDB. Memory types map to
        categories. Hebbian edge weights and decay state won't transfer.
      </p>

      <h2>Our Recommendation</h2>
      <p>
        There's no single "best" choice—it depends on your constraints:
      </p>

      <div className="not-prose grid md:grid-cols-3 gap-4 my-8">
        <div className="bg-blue-50 dark:bg-blue-900/20 rounded-lg p-4">
          <h4 className="font-semibold text-blue-900 dark:text-blue-100 mb-2">Cloud-First Teams</h4>
          <p className="text-sm text-blue-800 dark:text-blue-200">
            Mem0 or Zep. Both are mature, well-integrated, and handle infrastructure for you.
            Mem0 if you're on AWS; Zep if you need temporal features.
          </p>
        </div>
        <div className="bg-green-50 dark:bg-green-900/20 rounded-lg p-4">
          <h4 className="font-semibold text-green-900 dark:text-green-100 mb-2">Edge/Robotics</h4>
          <p className="text-sm text-green-800 dark:text-green-200">
            Shodh is the only option that runs fully offline on constrained devices.
            15MB binary, no network required, sub-ms latency.
          </p>
        </div>
        <div className="bg-purple-50 dark:bg-purple-900/20 rounded-lg p-4">
          <h4 className="font-semibold text-purple-900 dark:text-purple-100 mb-2">Privacy-Sensitive</h4>
          <p className="text-sm text-purple-800 dark:text-purple-200">
            Shodh—data never leaves the device. No cloud accounts, no API keys,
            no data transmission. Full sovereignty.
          </p>
        </div>
      </div>

      <h2>Getting Started</h2>

      <h3>Try Mem0</h3>
      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`pip install mem0ai
# Requires API key from mem0.ai`}
      />

      <h3>Try Zep</h3>
      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`pip install zep-cloud
# Requires API key from getzep.com`}
      />

      <h3>Try Shodh</h3>
      <CodeBlock
        language="bash"
        filename="Terminal"
        code={`pip install shodh-memory
# No API key needed - runs locally

# Or with Claude Code:
claude mcp add @shodh/memory-mcp`}
      />

      <h2>Conclusion</h2>
      <p>
        The AI memory space is maturing rapidly. Mem0 has the funding and AWS partnership.
        Zep has the temporal reasoning niche. Shodh has the edge/offline niche.
      </p>
      <p>
        If you're building cloud chatbots, Mem0 or Zep will serve you well. If you're
        building robots, drones, or privacy-first applications, Shodh is purpose-built
        for your constraints.
      </p>
      <p>
        The market is big enough for all three. Choose based on your deployment model,
        not marketing.
      </p>
    </article>
  );
}

// Main page component
export default function BlogPostPage() {
  const params = useParams();
  const slug = params.slug as string;
  const post = getPostBySlug(slug);

  if (!post) {
    return (
      <div className="min-h-screen flex items-center justify-center">
        <div className="text-center">
          <h1 className="text-4xl font-bold text-slate-900 dark:text-white mb-4">Post Not Found</h1>
          <Link href="/blog" className="text-orange-600 hover:underline">
            Back to Blog
          </Link>
        </div>
      </div>
    );
  }

  // Render the appropriate content based on slug
  const renderContent = () => {
    switch (slug) {
      case 'ai-memory-comparison-shodh-vs-mem0-vs-zep-2025':
        return <AIMemoryComparisonPost />;
      case 'shodh-memory-python-sdk-complete-tutorial':
        return <PythonSDKTutorialPost />;
      case 'how-shodh-memory-works-hebbian-learning':
        return <HebbianLearningPost />;
      case 'ai-memory-use-cases-developer-workflows':
        return <AIMemoryUseCasesPost />;
      case 'claude-code-mcp-server-setup-guide':
        return <ClaudeCodeMCPSetupPost />;
      case 'agentic-ai-long-term-memory-2025':
        return <AgenticAIMemoryPost />;
      case 'embodied-ai-edge-memory-robotics':
        return <EmbodiedAIMemoryPost />;
      case 'autonomous-robot-memory-ros2-guide':
        return <ROS2TutorialPost />;
      default:
        return <p>Content coming soon...</p>;
    }
  };

  return (
    <>
      <ArticleStructuredData post={post} />
      <div className="min-h-screen bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-800">
        <div className="max-w-4xl mx-auto px-4 py-12">
          {/* Back link */}
          <Link
          href="/blog"
          className="inline-flex items-center gap-2 text-slate-600 dark:text-slate-400 hover:text-orange-600 dark:hover:text-orange-400 mb-8"
        >
          <ArrowLeft className="w-4 h-4" />
          Back to Blog
        </Link>

        {/* Header */}
        <motion.header
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          className="mb-12"
        >
          <span className={`inline-block px-3 py-1 rounded-full text-sm font-medium mb-4 ${getCategoryColor(post.category)}`}>
            {post.category}
          </span>

          <h1 className="text-3xl md:text-4xl font-bold text-slate-900 dark:text-white mb-6">
            {post.title}
          </h1>

          <div className="flex flex-wrap items-center gap-4 text-sm text-slate-600 dark:text-slate-400 mb-6">
            <span className="flex items-center gap-1">
              <Calendar className="w-4 h-4" />
              {formatDate(post.date)}
            </span>
            <span className="flex items-center gap-1">
              <Clock className="w-4 h-4" />
              {post.readingTime}
            </span>
            <span>
              By {post.author.name}
              {post.author.role && ` · ${post.author.role}`}
            </span>
          </div>

          <div className="flex flex-wrap gap-2">
            {post.tags.map((tag) => (
              <span
                key={tag}
                className="inline-flex items-center gap-1 px-3 py-1 bg-slate-100 dark:bg-slate-800 text-slate-600 dark:text-slate-300 text-sm rounded-full"
              >
                <Tag className="w-3 h-3" />
                {tag}
              </span>
            ))}
          </div>
        </motion.header>

        {/* Content */}
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.5, delay: 0.2 }}
        >
          {renderContent()}
        </motion.div>

        {/* Footer */}
        <footer className="mt-16 pt-8 border-t border-slate-200 dark:border-slate-700">
          <div className="flex items-center justify-between">
            <Link
              href="/blog"
              className="text-orange-600 dark:text-orange-400 hover:underline"
            >
              ← More posts
            </Link>
            <a
              href="https://github.com/varun29ankuS/shodh-memory"
              target="_blank"
              rel="noopener noreferrer"
              className="text-slate-600 dark:text-slate-400 hover:text-orange-600 dark:hover:text-orange-400"
            >
              View on GitHub →
            </a>
          </div>
          </footer>
        </div>
      </div>
    </>
  );
}
