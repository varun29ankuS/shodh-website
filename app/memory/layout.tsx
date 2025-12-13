import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Shodh Memory - Cognitive Memory for AI Agents | Claude, GPT, LangChain',
  description: 'Persistent memory for AI agents that learns with use. Hebbian learning, 3-tier cognitive architecture, knowledge graphs. Works with Claude Code, Claude Desktop, Cursor, LangChain, AutoGPT. Single ~15MB binary, 100% offline.',
  keywords: [
    'AI agent memory', 'persistent memory', 'Claude memory', 'GPT memory',
    'LangChain memory', 'AutoGPT memory', 'MCP memory', 'cognitive memory',
    'Hebbian learning', 'knowledge graph', 'offline AI', 'local memory',
    'Claude Code', 'Claude Desktop', 'Cursor IDE', 'AI assistant memory',
    'semantic memory', 'episodic memory', 'spreading activation',
    'long-term potentiation', 'memory consolidation', 'Rust AI'
  ],
  openGraph: {
    title: 'Shodh Memory - Cognitive Memory for AI Agents',
    description: 'Give Claude, GPT, and your custom agents persistent memory that strengthens with use. Hebbian learning, 3-tier architecture, knowledge graphs. 100% offline.',
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
    title: 'Shodh Memory - Cognitive Memory for AI Agents',
    description: 'Persistent memory for Claude, GPT & custom agents. Hebbian learning, knowledge graphs. Single binary, 100% offline.',
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
