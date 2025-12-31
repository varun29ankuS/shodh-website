'use client'

import { motion } from 'framer-motion'
import {
  ArrowRight, Brain, Database, Lock, Zap, Clock, Network,
  Shield, Cpu, GitBranch, Code, Bot, Sparkles, Terminal,
  Layers, RefreshCw, Search, MessageSquare, Workflow,
  CheckSquare, Bell, FolderOpen, Monitor, ListTodo
} from 'lucide-react'
import Link from 'next/link'
import NeuralAnimation from '@/components/NeuralAnimation'

export default function Memory() {
  const keyFeatures = [
    {
      icon: Brain,
      title: 'Hebbian Learning',
      description: 'Memories strengthen when retrieved together - just like biological synapses',
      detail: 'Co-activated memories form edges; 5+ co-activations become permanent',
    },
    {
      icon: Layers,
      title: '3-Tier Cognitive Architecture',
      description: 'Working → Session → Long-term memory based on Cowan\'s model',
      detail: 'Sub-ms working memory, automatic promotion based on importance',
    },
    {
      icon: Network,
      title: 'Knowledge Graph',
      description: 'Entities, relationships, and spreading activation retrieval',
      detail: '763ns entity lookup, 32μs 3-hop traversal',
    },
    {
      icon: Lock,
      title: '100% Local & Private',
      description: 'Single ~18MB binary. No cloud. No data leaves your machine.',
      detail: 'RocksDB storage, works completely offline',
    },
    {
      icon: Zap,
      title: 'Sub-millisecond Graph Ops',
      description: 'Rust-native performance for real-time agent decisions',
      detail: '~55ms store (with embedding), ~45ms semantic search',
    },
    {
      icon: Search,
      title: 'Hybrid Retrieval',
      description: 'Semantic, associative, and hybrid search modes',
      detail: 'Density-dependent weighting adapts to your usage patterns',
    },
  ]

  const integrations = [
    {
      icon: Terminal,
      title: 'Claude Code',
      description: 'Add persistent memory to your CLI coding sessions',
      setup: 'One line in claude_code_config.json',
    },
    {
      icon: MessageSquare,
      title: 'Claude Desktop',
      description: 'Remember conversations across chat sessions',
      setup: 'One line in claude_desktop_config.json',
    },
    {
      icon: Code,
      title: 'Cursor IDE',
      description: 'Give your AI coding assistant project memory',
      setup: 'Add to ~/.cursor/mcp.json',
    },
    {
      icon: Workflow,
      title: 'LangChain / AutoGPT',
      description: 'Persistent memory for autonomous agents',
      setup: 'Python SDK or REST API',
    },
  ]

  const comparisonData = [
    { feature: 'Store memory', shodh: '55-60ms', mem0: '100-500ms', langchain: 'Varies' },
    { feature: 'Semantic search', shodh: '34-58ms', mem0: '200-500ms', langchain: 'Varies' },
    { feature: 'Graph operations', shodh: '<1μs', mem0: 'N/A', langchain: 'N/A' },
    { feature: 'Deployment', shodh: 'Local binary', mem0: 'Cloud API', langchain: 'Cloud DBs' },
    { feature: 'Offline support', shodh: '100%', mem0: 'No', langchain: 'Partial' },
    { feature: 'Learning algorithm', shodh: 'Hebbian + LTP', mem0: 'None', langchain: 'None' },
    { feature: 'MCP support', shodh: 'Native (37 tools)', mem0: 'No', langchain: 'No' },
    { feature: 'Task management', shodh: 'Built-in', mem0: 'No', langchain: 'No' },
    { feature: 'Reminders', shodh: 'Built-in', mem0: 'No', langchain: 'No' },
    { feature: 'Proactive context', shodh: 'Yes', mem0: 'No', langchain: 'No' },
    { feature: 'Binary size', shodh: '~18MB', mem0: 'Cloud', langchain: 'Cloud' },
  ]

  const benchmarks = [
    { operation: 'Entity Get (1000)', latency: '810ns', note: 'O(1) hash lookup' },
    { operation: 'Entity Search', latency: '990ns', note: 'Sub-microsecond' },
    { operation: 'Relationship Query', latency: '2.3μs', note: 'Graph traversal' },
    { operation: '3-hop Traversal', latency: '32.8μs', note: 'Deep graph walk' },
    { operation: 'Hebbian Strengthen', latency: '6.2μs', note: 'Synapse update' },
    { operation: 'Remember (full)', latency: '55-60ms', note: 'Embedding + NER + store' },
    { operation: 'Recall (semantic)', latency: '34-58ms', note: 'Vector search + rank' },
    { operation: 'Recall (tags)', latency: '~1ms', note: 'Index lookup' },
  ]

  const mcpTools = [
    {
      category: 'Memory',
      tools: ['remember', 'recall', 'recall_by_tags', 'recall_by_date', 'forget', 'forget_by_tags', 'forget_by_date', 'list_memories', 'context_summary', 'proactive_context'],
    },
    {
      category: 'Todos',
      tools: ['add_todo', 'list_todos', 'update_todo', 'complete_todo', 'delete_todo', 'reorder_todo', 'list_subtasks', 'todo_stats'],
    },
    {
      category: 'Projects',
      tools: ['add_project', 'list_projects', 'archive_project', 'delete_project'],
    },
    {
      category: 'Reminders',
      tools: ['set_reminder', 'list_reminders', 'dismiss_reminder'],
    },
    {
      category: 'System',
      tools: ['memory_stats', 'verify_index', 'repair_index', 'consolidation_report', 'streaming_status', 'token_status', 'reset_token_session'],
    },
  ]

  return (
    <main className="min-h-screen overflow-hidden">
      {/* Hero */}
      <section className="relative min-h-screen flex items-center justify-center bg-gradient-to-b from-slate-50 via-white to-slate-50 dark:from-slate-950 dark:via-slate-900 dark:to-slate-950 overflow-hidden">
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute -top-40 -left-40 w-96 h-96 bg-primary/30 rounded-full blur-3xl animate-pulse"></div>
          <div className="absolute top-1/3 -right-40 w-[600px] h-[600px] bg-secondary/25 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '1s' }}></div>
          <div className="absolute -bottom-40 left-1/3 w-[500px] h-[500px] bg-destructive/30 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '2s' }}></div>
        </div>
        <div className="absolute inset-0 bg-grid-slate opacity-40" />
        <div className="absolute inset-0 gradient-mesh"></div>

        {/* Neural network animation - interactive cognition visualization */}
        <NeuralAnimation />

        <div className="container mx-auto px-4 py-20 relative z-10">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            className="max-w-5xl mx-auto text-center"
          >
            <motion.div
              initial={{ opacity: 0, scale: 0.95 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.1 }}
              className="inline-flex items-center gap-2 px-4 py-2 bg-primary/10 dark:bg-primary/20 rounded-full mb-8"
            >
              <Brain className="w-4 h-4 text-primary" />
              <span className="text-sm font-semibold text-primary">The Missing Brain for Stateless LLMs</span>
            </motion.div>

            <h1 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white">
              Memory That Learns
              <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
                For Claude, GPT & Your Agents
              </span>
            </h1>

            <p className="text-xl md:text-2xl text-slate-600 dark:text-slate-400 mb-8 max-w-3xl mx-auto">
              LLMs are stateless. Every conversation starts from scratch.
              Shodh Memory gives them persistent memory that strengthens with use.
              Hebbian learning, spreading activation, 3-tier cognitive architecture.
            </p>

            <p className="text-base text-slate-500 dark:text-slate-500 mb-12 max-w-2xl mx-auto">
              <strong className="text-primary">One-liner setup</strong> for Claude Code & Desktop.
              <strong className="text-primary ml-2">37 MCP tools</strong> for memory, todos, reminders.
              <strong className="text-primary ml-2">Python SDK</strong> for custom agents.
            </p>

            <div className="flex flex-col sm:flex-row gap-4 justify-center mb-12">
              <Link
                href="/getting-started"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Get Started
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <Link
                href="/docs"
                className="px-8 py-4 border-2 border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                Documentation
              </Link>
            </div>

            {/* Quick Stats */}
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 max-w-3xl mx-auto">
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.3 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">759</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Tests Passing</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.4 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">37</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">MCP Tools</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.5 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">66k+</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Lines of Code</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.6 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">100%</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Offline</div>
              </motion.div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* LLMs Are Stateless Problem Statement */}
      <section className="relative py-24 bg-gradient-to-b from-slate-100 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="max-w-4xl mx-auto">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="text-center mb-12"
            >
              <h2 className="text-4xl md:text-5xl font-bold mb-6 text-slate-900 dark:text-white">
                The Problem: LLMs Are Stateless
              </h2>
              <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
                Every conversation with Claude or GPT starts from scratch. They forget what you told them yesterday.
                They can't learn from past mistakes. They rediscover the same patterns again and again.
              </p>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="grid md:grid-cols-2 gap-6"
            >
              <div className="p-6 bg-red-50 dark:bg-red-950/30 rounded-xl border border-red-200 dark:border-red-900">
                <h3 className="text-lg font-semibold text-red-700 dark:text-red-400 mb-4">Without Memory</h3>
                <ul className="space-y-3 text-slate-700 dark:text-slate-300 text-sm">
                  <li className="flex items-start gap-2">
                    <span className="text-red-500">✗</span>
                    <span>Repeats same mistakes across sessions</span>
                  </li>
                  <li className="flex items-start gap-2">
                    <span className="text-red-500">✗</span>
                    <span>Forgets user preferences and context</span>
                  </li>
                  <li className="flex items-start gap-2">
                    <span className="text-red-500">✗</span>
                    <span>Can't learn from successful patterns</span>
                  </li>
                  <li className="flex items-start gap-2">
                    <span className="text-red-500">✗</span>
                    <span>No accumulated project knowledge</span>
                  </li>
                </ul>
              </div>
              <div className="p-6 bg-green-50 dark:bg-green-950/30 rounded-xl border border-green-200 dark:border-green-900">
                <h3 className="text-lg font-semibold text-green-700 dark:text-green-400 mb-4">With Shodh Memory</h3>
                <ul className="space-y-3 text-slate-700 dark:text-slate-300 text-sm">
                  <li className="flex items-start gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Remembers errors and avoids them</span>
                  </li>
                  <li className="flex items-start gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Builds persistent user model</span>
                  </li>
                  <li className="flex items-start gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Strengthens useful associations (Hebbian)</span>
                  </li>
                  <li className="flex items-start gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Accumulates domain expertise over time</span>
                  </li>
                </ul>
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* MCP Integration - The Key Feature */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              One Line to Remember Everything
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Add persistent memory to Claude Code, Claude Desktop, or Cursor with a single config line
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto p-8 bg-slate-900 dark:bg-slate-950 rounded-2xl border border-slate-700"
          >
            <div className="flex items-center gap-2 mb-6 text-slate-400">
              <Terminal className="w-5 h-5" />
              <span className="text-sm font-semibold">MCP Configuration - Works Instantly</span>
            </div>
            <pre className="text-slate-100 font-mono text-sm overflow-x-auto mb-6">
{`// Add to your MCP config file
{
  "mcpServers": {
    "shodh-memory": {
      "command": "npx",
      "args": ["-y", "@shodh/memory-mcp"],
      "env": {
        "SHODH_API_KEY": "your-api-key"
      }
    }
  }
}`}
            </pre>

            <div className="grid md:grid-cols-3 gap-4 mt-6 text-sm">
              <div className="p-4 bg-slate-800 rounded-lg">
                <div className="text-primary font-semibold mb-1">Claude Code</div>
                <code className="text-xs text-slate-400">~/.claude/claude_code_config.json</code>
              </div>
              <div className="p-4 bg-slate-800 rounded-lg">
                <div className="text-primary font-semibold mb-1">Claude Desktop</div>
                <code className="text-xs text-slate-400">claude_desktop_config.json</code>
              </div>
              <div className="p-4 bg-slate-800 rounded-lg">
                <div className="text-primary font-semibold mb-1">Cursor</div>
                <code className="text-xs text-slate-400">~/.cursor/mcp.json</code>
              </div>
            </div>

            <div className="mt-6 p-4 bg-primary/10 dark:bg-primary/20 rounded-lg border-l-4 border-primary">
              <p className="text-sm text-slate-300">
                <strong className="text-primary">What you get:</strong> Claude remembers your decisions, learned patterns,
                errors to avoid, and project context across all sessions. Memories strengthen when retrieved together (Hebbian learning).
              </p>
            </div>
          </motion.div>

          {/* Integration Cards */}
          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6 max-w-6xl mx-auto mt-12">
            {integrations.map((integration, index) => (
              <motion.div
                key={integration.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all"
              >
                <div className="w-10 h-10 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <integration.icon className="w-5 h-5 text-primary" />
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">{integration.title}</h3>
                <p className="text-sm text-slate-600 dark:text-slate-400 mb-2">{integration.description}</p>
                <p className="text-xs text-primary font-medium">{integration.setup}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* 37 MCP Tools */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              37 MCP Tools
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              More than just memory - a complete cognitive toolkit for AI agents
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-5xl mx-auto grid md:grid-cols-2 lg:grid-cols-3 gap-6"
          >
            {mcpTools.map((category, index) => (
              <div
                key={category.category}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <div className="flex items-center gap-2 mb-4">
                  {category.category === 'Memory' && <Brain className="w-5 h-5 text-primary" />}
                  {category.category === 'Todos' && <CheckSquare className="w-5 h-5 text-primary" />}
                  {category.category === 'Projects' && <FolderOpen className="w-5 h-5 text-primary" />}
                  {category.category === 'Reminders' && <Bell className="w-5 h-5 text-primary" />}
                  {category.category === 'System' && <Cpu className="w-5 h-5 text-primary" />}
                  <h3 className="text-lg font-semibold text-slate-900 dark:text-white">{category.category}</h3>
                  <span className="ml-auto text-xs bg-primary/10 text-primary px-2 py-1 rounded-full">{category.tools.length}</span>
                </div>
                <div className="flex flex-wrap gap-2">
                  {category.tools.map((tool) => (
                    <code key={tool} className="text-xs bg-slate-100 dark:bg-slate-800 text-slate-700 dark:text-slate-300 px-2 py-1 rounded">
                      {tool}
                    </code>
                  ))}
                </div>
              </div>
            ))}
          </motion.div>
        </div>
      </section>

      {/* Not Just Memory - Productivity Features */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Not Just Memory
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Built-in task management, reminders, and a TUI dashboard for introspection
            </p>
          </div>

          <div className="max-w-6xl mx-auto grid md:grid-cols-2 lg:grid-cols-4 gap-6">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-slate-50 dark:bg-slate-900"
            >
              <ListTodo className="w-10 h-10 text-primary mb-4" />
              <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">GTD Todos</h3>
              <p className="text-sm text-slate-600 dark:text-slate-400">
                Full GTD workflow with projects, contexts (@computer, @phone), priorities, due dates, and subtasks.
              </p>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-slate-50 dark:bg-slate-900"
            >
              <Bell className="w-10 h-10 text-primary mb-4" />
              <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">Smart Reminders</h3>
              <p className="text-sm text-slate-600 dark:text-slate-400">
                Time-based, duration-based, or context-triggered reminders that surface when relevant keywords appear.
              </p>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.2 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-slate-50 dark:bg-slate-900"
            >
              <FolderOpen className="w-10 h-10 text-primary mb-4" />
              <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">Projects</h3>
              <p className="text-sm text-slate-600 dark:text-slate-400">
                Organize todos into projects with sub-project support. Archive when done. Codebase indexing for file context.
              </p>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.3 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-slate-50 dark:bg-slate-900"
            >
              <Monitor className="w-10 h-10 text-primary mb-4" />
              <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">TUI Dashboard</h3>
              <p className="text-sm text-slate-600 dark:text-slate-400">
                Terminal UI for memory introspection. Watch memories form, see Hebbian strengthening, browse knowledge graph.
              </p>
            </motion.div>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto mt-12 p-6 bg-primary/5 dark:bg-primary/10 rounded-xl border border-primary/20"
          >
            <div className="flex items-start gap-4">
              <Sparkles className="w-8 h-8 text-primary flex-shrink-0 mt-1" />
              <div>
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">
                  Proactive Context
                </h4>
                <p className="text-sm text-slate-700 dark:text-slate-300">
                  The <code className="bg-slate-200 dark:bg-slate-800 px-1 rounded">proactive_context</code> tool automatically surfaces relevant memories
                  based on the current conversation. Claude doesn't need to search - context appears when it's needed.
                  This is how biological memory works: associations activate related memories without conscious effort.
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Python SDK */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-12">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Python SDK for Custom Agents
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Build agents that learn from experience
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto p-8 bg-slate-900 dark:bg-slate-950 rounded-2xl border border-slate-700"
          >
            <div className="flex items-center gap-2 mb-6 text-slate-400">
              <Code className="w-5 h-5" />
              <span className="text-sm font-semibold">pip install shodh-memory</span>
            </div>
            <pre className="text-slate-100 font-mono text-sm overflow-x-auto">
{`from shodh_memory import Memory

# Initialize (auto-starts server if needed)
memory = Memory(api_key="your-api-key", storage_path="./my_agent_memory")

# Store memories with types for importance weighting
memory.remember("User prefers dark mode", memory_type="Decision")
memory.remember("JWT tokens expire after 24h", memory_type="Learning")
memory.remember("Don't use deprecated API v1", memory_type="Error")

# Semantic search - finds related memories
results = memory.recall("user preferences", limit=5)
for mem in results:
    print(f"{mem['content']} (importance: {mem.get('importance', 0):.2f})")

# Hybrid retrieval - semantic + graph associations
results = memory.recall("authentication", mode="hybrid", limit=10)

# Get context summary for LLM bootstrap
summary = memory.context_summary()
# Returns: decisions, learnings, errors, patterns - structured for prompts

# Memory statistics
stats = memory.get_stats()
print(f"Total memories: {stats['total_memories']}")`}
            </pre>

            <div className="grid md:grid-cols-2 gap-4 mt-6">
              <div className="p-4 bg-primary/10 dark:bg-primary/20 rounded-lg">
                <p className="text-xs text-slate-300">
                  <strong className="text-primary">Memory Types:</strong> Decision (+0.30), Learning (+0.25),
                  Error (+0.25), Discovery (+0.20), Task (+0.15), Context (+0.10)
                </p>
              </div>
              <div className="p-4 bg-secondary/10 dark:bg-secondary/20 rounded-lg">
                <p className="text-xs text-slate-300">
                  <strong className="text-secondary">Retrieval Modes:</strong> semantic (vector),
                  associative (graph), hybrid (density-weighted combination)
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Key Features */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Cognitive Architecture
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Based on neuroscience research - Cowan's working memory model, Hebbian plasticity, spreading activation
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {keyFeatures.map((feature, index) => (
              <motion.div
                key={feature.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="group p-6 rounded-xl border border-slate-200 dark:border-slate-800 hover:border-primary dark:hover:border-primary transition-all duration-300 bg-slate-50 dark:bg-slate-900 hover:shadow-xl hover:shadow-primary/10 hover:-translate-y-1"
              >
                <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <feature.icon className="w-6 h-6 text-primary" />
                </div>
                <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">{feature.title}</h3>
                <p className="text-slate-600 dark:text-slate-400 mb-3">{feature.description}</p>
                <p className="text-xs text-primary font-semibold">{feature.detail}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Architecture Diagram */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              How It Works
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              3-tier memory + knowledge graph + Hebbian learning
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-5xl mx-auto"
          >
            <div className="p-8 bg-slate-900 dark:bg-slate-950 rounded-2xl border border-slate-700 mb-8">
              <pre className="text-slate-100 font-mono text-xs md:text-sm overflow-x-auto" style={{ fontFamily: 'Courier New, Consolas, monospace', lineHeight: '1.5', whiteSpace: 'pre' }}>
{`┌──────────────────────────────────────────────────────────────────┐
│                     SHODH MEMORY ARCHITECTURE                     │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │              WORKING MEMORY (LRU Cache)                    │  │
│  │  • 100 most recent/frequent memories                       │  │
│  │  • Retrieval: <1ms                                         │  │
│  │  • High importance items promoted here                     │  │
│  └────────────────────────────────────────────────────────────┘  │
│                          ↓ overflow ↑ access                     │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │              SESSION MEMORY (Bounded)                      │  │
│  │  • Current session context                                 │  │
│  │  • Retrieval: <10ms                                        │  │
│  │  • Promotion threshold: importance > 0.6                   │  │
│  └────────────────────────────────────────────────────────────┘  │
│                          ↓ consolidation ↑ recall                │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │              LONG-TERM MEMORY (RocksDB)                    │  │
│  │  • Persistent storage with LZ4 compression                 │  │
│  │  • Retrieval: <100ms                                       │  │
│  │  • Semantic consolidation after 7+ days                    │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │              KNOWLEDGE GRAPH (Hebbian)                     │  │
│  │                                                            │  │
│  │  Memory A ──co-retrieved──▶ Memory B                       │  │
│  │      │                          │                          │  │
│  │      └────strengthens───────────┘                          │  │
│  │                                                            │  │
│  │  • Edges form on co-retrieval (Hebbian learning)           │  │
│  │  • 5+ co-activations = Long-Term Potentiation (permanent)  │  │
│  │  • Unused edges decay over time                            │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │              VECTOR INDEX (Vamana HNSW)                    │  │
│  │  • MiniLM-L6-v2 embeddings (384-dim, 25MB model)           │  │
│  │  • TinyBERT NER for entity extraction (15MB model)         │  │
│  │  • Approximate nearest neighbor search                     │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘`}
              </pre>
            </div>

            <div className="grid md:grid-cols-3 gap-4">
              <div className="p-6 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <RefreshCw className="w-8 h-8 text-primary mb-3" />
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">Automatic Consolidation</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  Important memories promote up the hierarchy; old memories compress into semantic facts
                </p>
              </div>
              <div className="p-6 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <GitBranch className="w-8 h-8 text-primary mb-3" />
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">Spreading Activation</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  Queries activate related memories through graph connections, not just vector similarity
                </p>
              </div>
              <div className="p-6 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <Database className="w-8 h-8 text-primary mb-3" />
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">Durable Storage</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  RocksDB with LZ4 compression. Survives restarts. Works offline.
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Benchmarks */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Benchmark Results
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Criterion benchmarks on Intel i7-1355U, release build
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto overflow-x-auto"
          >
            <table className="w-full border-collapse">
              <thead>
                <tr className="border-b-2 border-slate-200 dark:border-slate-800">
                  <th className="text-left p-4 text-slate-900 dark:text-white font-semibold">Operation</th>
                  <th className="text-center p-4 text-primary font-semibold">Latency</th>
                  <th className="text-left p-4 text-slate-600 dark:text-slate-400 font-semibold">Notes</th>
                </tr>
              </thead>
              <tbody>
                {benchmarks.map((row, index) => (
                  <tr
                    key={row.operation}
                    className={`border-b border-slate-100 dark:border-slate-800 ${
                      index % 2 === 0 ? 'bg-slate-50 dark:bg-slate-900/50' : ''
                    }`}
                  >
                    <td className="p-4 text-slate-900 dark:text-white font-medium">{row.operation}</td>
                    <td className="p-4 text-center text-primary font-semibold font-mono">{row.latency}</td>
                    <td className="p-4 text-slate-600 dark:text-slate-400 text-sm">{row.note}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </motion.div>
        </div>
      </section>

      {/* Comparison */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              vs. Alternatives
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Local-first alternative to cloud memory services
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-5xl mx-auto overflow-x-auto"
          >
            <table className="w-full border-collapse">
              <thead>
                <tr className="border-b-2 border-slate-200 dark:border-slate-800">
                  <th className="text-left p-4 text-slate-900 dark:text-white font-semibold">Feature</th>
                  <th className="text-center p-4 text-primary font-semibold">Shodh Memory</th>
                  <th className="text-center p-4 text-slate-600 dark:text-slate-400 font-semibold">Mem0</th>
                  <th className="text-center p-4 text-slate-600 dark:text-slate-400 font-semibold">LangChain Memory</th>
                </tr>
              </thead>
              <tbody>
                {comparisonData.map((row, index) => (
                  <tr
                    key={row.feature}
                    className={`border-b border-slate-100 dark:border-slate-800 ${
                      index % 2 === 0 ? 'bg-slate-50 dark:bg-slate-900/50' : ''
                    }`}
                  >
                    <td className="p-4 text-slate-900 dark:text-white font-medium">{row.feature}</td>
                    <td className="p-4 text-center text-primary font-semibold">{row.shodh}</td>
                    <td className="p-4 text-center text-slate-600 dark:text-slate-400">{row.mem0}</td>
                    <td className="p-4 text-center text-slate-600 dark:text-slate-400">{row.langchain}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </motion.div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-3xl mx-auto mt-12 p-6 bg-primary/5 dark:bg-primary/10 rounded-xl border border-primary/20"
          >
            <div className="flex items-start gap-4">
              <Shield className="w-8 h-8 text-primary flex-shrink-0 mt-1" />
              <div>
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">
                  Why Local Matters
                </h4>
                <p className="text-sm text-slate-700 dark:text-slate-300">
                  Your agent's memories contain sensitive context - user preferences, business logic,
                  learned patterns. Shodh Memory keeps everything on your machine. No cloud API calls,
                  no data transmission, no vendor lock-in. Works completely offline.
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* CTA */}
      <section className="relative py-24 bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20 overflow-hidden">
        <div className="absolute inset-0">
          <div className="absolute top-0 right-0 w-96 h-96 bg-primary/30 rounded-full blur-3xl animate-pulse"></div>
          <div className="absolute bottom-0 left-0 w-96 h-96 bg-secondary/30 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '1s' }}></div>
        </div>
        <div className="container mx-auto px-4 relative z-10">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto text-center"
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6 text-slate-900 dark:text-white">
              Give Your AI a Brain
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
              One-liner for Claude. Python SDK for custom agents. REST API for everything else.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Link
                href="/getting-started"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Get Started
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <a
                href="https://github.com/varun29ankuS/shodh-memory"
                target="_blank"
                rel="noopener noreferrer"
                className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                View on GitHub
              </a>
            </div>

            <div className="mt-12 grid grid-cols-3 gap-8 max-w-md mx-auto text-sm">
              <a href="https://pypi.org/project/shodh-memory/" target="_blank" rel="noopener noreferrer" className="text-slate-600 dark:text-slate-400 hover:text-primary transition-colors">
                PyPI
              </a>
              <a href="https://www.npmjs.com/package/@shodh/memory-mcp" target="_blank" rel="noopener noreferrer" className="text-slate-600 dark:text-slate-400 hover:text-primary transition-colors">
                npm
              </a>
              <a href="https://crates.io/crates/shodh-memory" target="_blank" rel="noopener noreferrer" className="text-slate-600 dark:text-slate-400 hover:text-primary transition-colors">
                crates.io
              </a>
            </div>
          </motion.div>
        </div>
      </section>
    </main>
  )
}
