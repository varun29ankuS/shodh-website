import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Documentation - Quick Start & API Reference',
  description: 'Get started with Shodh Memory in 2 minutes. pip install shodh-memory. Python SDK, MCP server for Claude Code, REST API. Full documentation and examples.',
  keywords: [
    'shodh memory documentation', 'AI memory API', 'MCP server setup',
    'Claude Code memory', 'Python AI SDK', 'persistent AI memory tutorial'
  ],
  openGraph: {
    title: 'Shodh Documentation - Quick Start Guide',
    description: 'Install in 30 seconds: pip install shodh-memory. Python SDK, MCP server, REST API.',
    url: 'https://shodh-rag.com/docs',
  },
}

export default function DocsLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
