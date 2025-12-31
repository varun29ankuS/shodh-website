import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Shodh Memory - The Missing Brain for Stateless LLMs | 37 MCP Tools',
  description: 'Persistent memory for AI agents that learns with use. 37 MCP tools: memory, todos, reminders, projects. Hebbian learning, 3-tier cognitive architecture, knowledge graphs. Works with Claude Code, Claude Desktop, Cursor. Single ~18MB binary, 100% offline.',
  keywords: [
    'AI agent memory', 'persistent memory', 'Claude memory', 'GPT memory',
    'LangChain memory', 'AutoGPT memory', 'MCP memory', 'cognitive memory',
    'Hebbian learning', 'knowledge graph', 'offline AI', 'local memory',
    'Claude Code', 'Claude Desktop', 'Cursor IDE', 'AI assistant memory',
    'semantic memory', 'episodic memory', 'spreading activation',
    'long-term potentiation', 'memory consolidation', 'Rust AI',
    'MCP tools', 'AI todos', 'AI reminders', 'proactive context', 'TUI dashboard'
  ],
  openGraph: {
    title: 'Shodh Memory - The Missing Brain for Stateless LLMs',
    description: '37 MCP tools for Claude, GPT & custom agents. Memory, todos, reminders, projects. Hebbian learning that strengthens with use. 100% offline.',
    type: 'website',
    url: 'https://shodh-rag.com/memory',
    images: [
      {
        url: '/og-memory.png',
        width: 1200,
        height: 630,
        alt: 'Shodh Memory - Memory That Learns for Claude, GPT & Your Agents',
      },
    ],
  },
  twitter: {
    card: 'summary_large_image',
    title: 'Shodh Memory - The Missing Brain for Stateless LLMs',
    description: '37 MCP tools: memory, todos, reminders, projects. Hebbian learning, knowledge graphs. Single ~18MB binary, 100% offline.',
    images: ['/og-memory.png'],
  },
  alternates: {
    canonical: 'https://shodh-rag.com/memory',
  },
}

export default function MemoryLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
