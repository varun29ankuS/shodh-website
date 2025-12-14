'use client'

import { motion } from 'framer-motion'
import Link from 'next/link'
import {
  Book, Code, Terminal, Rocket, Github, ExternalLink,
  Play, Package, MessageCircle, FileText, Cpu, Brain,
  ArrowRight, Copy, Check, Database
} from 'lucide-react'
import { useState } from 'react'

function CopyButton({ text }: { text: string }) {
  const [copied, setCopied] = useState(false)

  const handleCopy = async () => {
    await navigator.clipboard.writeText(text)
    setCopied(true)
    setTimeout(() => setCopied(false), 2000)
  }

  return (
    <button
      onClick={handleCopy}
      className="p-2 hover:bg-slate-700 rounded transition-colors"
      title="Copy to clipboard"
    >
      {copied ? (
        <Check className="w-4 h-4 text-green-400" />
      ) : (
        <Copy className="w-4 h-4 text-slate-400" />
      )}
    </button>
  )
}

export default function Docs() {
  const quickStart = [
    {
      title: 'Python SDK',
      command: 'pip install shodh-memory',
      description: 'Full-featured Python library for memory operations',
      link: 'https://pypi.org/project/shodh-memory/',
      linkText: 'PyPI',
    },
    {
      title: 'MCP Server (npm)',
      command: 'npx @shodh/memory-mcp',
      description: 'Model Context Protocol server for Claude Code/Desktop',
      link: 'https://www.npmjs.com/package/@shodh/memory-mcp',
      linkText: 'npm',
    },
  ]

  const resources = [
    {
      icon: Github,
      title: 'GitHub Repository',
      description: 'Source code, README, issues, and discussions',
      link: 'https://github.com/varun29ankuS/shodh-memory',
      primary: true,
    },
    {
      icon: Play,
      title: 'Interactive Colab',
      description: 'Try all features in your browser - no setup required',
      link: 'https://colab.research.google.com/github/varun29ankuS/shodh-memory/blob/main/notebooks/shodh_memory_demo.ipynb',
      primary: true,
    },
    {
      icon: Package,
      title: 'PyPI Package',
      description: 'Python package with version history and metadata',
      link: 'https://pypi.org/project/shodh-memory/',
      primary: false,
    },
    {
      icon: Package,
      title: 'npm Package',
      description: 'MCP server package for Node.js environments',
      link: 'https://www.npmjs.com/package/@shodh/memory-mcp',
      primary: false,
    },
  ]

  const guides = [
    {
      icon: Rocket,
      title: 'Quick Start',
      description: 'Get up and running in 2 minutes',
      link: 'https://github.com/varun29ankuS/shodh-memory#quick-start',
    },
    {
      icon: Terminal,
      title: 'MCP Setup',
      description: 'Configure for Claude Code or Claude Desktop',
      link: 'https://github.com/varun29ankuS/shodh-memory#mcp-server',
    },
    {
      icon: Code,
      title: 'API Reference',
      description: 'Full Python API documentation',
      link: 'https://github.com/varun29ankuS/shodh-memory#api-reference',
    },
    {
      icon: Cpu,
      title: 'Technical Specs',
      description: 'Architecture, benchmarks, and deployment options',
      link: '/features',
      internal: true,
    },
  ]

  return (
    <main className="min-h-screen bg-gradient-to-b from-white via-slate-50 to-white dark:from-slate-950 dark:via-slate-900 dark:to-slate-950">
      <div className="container mx-auto px-4 py-24">
        {/* Header */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="text-center mb-16"
        >
          <div className="inline-flex items-center gap-2 px-4 py-2 bg-primary/10 dark:bg-primary/20 rounded-full mb-6">
            <Brain className="w-4 h-4 text-primary" />
            <span className="text-sm font-semibold text-primary">Shodh Memory</span>
          </div>
          <h1 className="text-5xl md:text-6xl font-bold mb-6 text-slate-900 dark:text-white">
            Documentation
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
            Everything you need to get started with Shodh Memory - the open-source cognitive memory system for AI agents
          </p>
        </motion.div>

        {/* RAG Note */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.05 }}
          className="max-w-3xl mx-auto mb-12 p-4 rounded-xl bg-secondary/5 dark:bg-secondary/10 border border-secondary/20"
        >
          <div className="flex items-start gap-3">
            <FileText className="w-5 h-5 text-secondary flex-shrink-0 mt-0.5" />
            <div>
              <p className="text-sm text-slate-700 dark:text-slate-300">
                <strong className="text-secondary">Looking for Shodh RAG docs?</strong>{' '}
                RAG documentation is provided during enterprise pilots.
                <a href="https://wa.me/919810300618?text=Hi, I'd like to access Shodh RAG documentation" target="_blank" rel="noopener noreferrer" className="text-secondary hover:underline ml-1">
                  Contact us to get started →
                </a>
              </p>
            </div>
          </div>
        </motion.div>

        {/* Quick Install */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.1 }}
          className="max-w-3xl mx-auto mb-16"
        >
          <h2 className="text-2xl font-bold mb-6 text-center text-slate-900 dark:text-white">
            Quick Install
          </h2>
          <div className="grid md:grid-cols-2 gap-4">
            {quickStart.map((item) => (
              <div
                key={item.title}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-semibold text-slate-900 dark:text-white">
                    {item.title}
                  </h3>
                  <a
                    href={item.link}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="text-xs text-primary hover:underline flex items-center gap-1"
                  >
                    {item.linkText} <ExternalLink className="w-3 h-3" />
                  </a>
                </div>
                <div className="flex items-center gap-2 p-3 bg-slate-900 dark:bg-slate-950 rounded-lg font-mono text-sm">
                  <code className="text-green-400 flex-1">{item.command}</code>
                  <CopyButton text={item.command} />
                </div>
                <p className="text-xs text-slate-500 dark:text-slate-400 mt-3">
                  {item.description}
                </p>
              </div>
            ))}
          </div>
        </motion.div>

        {/* Primary Resources */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-4xl mx-auto mb-16"
        >
          <h2 className="text-2xl font-bold mb-6 text-center text-slate-900 dark:text-white">
            Resources
          </h2>
          <div className="grid md:grid-cols-2 gap-4">
            {resources.map((resource, index) => (
              <motion.a
                key={resource.title}
                href={resource.link}
                target="_blank"
                rel="noopener noreferrer"
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.05 }}
                viewport={{ once: true }}
                className={`p-6 rounded-xl border-2 transition-all hover:shadow-lg group ${
                  resource.primary
                    ? 'border-primary/30 bg-gradient-to-br from-primary/5 to-white dark:from-primary/10 dark:to-slate-900 hover:border-primary'
                    : 'border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary'
                }`}
              >
                <div className="flex items-start gap-4">
                  <div className={`w-12 h-12 rounded-lg flex items-center justify-center flex-shrink-0 ${
                    resource.primary
                      ? 'bg-primary/20'
                      : 'bg-slate-100 dark:bg-slate-800'
                  }`}>
                    <resource.icon className={`w-6 h-6 ${
                      resource.primary ? 'text-primary' : 'text-slate-600 dark:text-slate-400'
                    }`} />
                  </div>
                  <div className="flex-1">
                    <div className="flex items-center gap-2">
                      <h3 className="font-semibold text-slate-900 dark:text-white">
                        {resource.title}
                      </h3>
                      <ExternalLink className="w-4 h-4 text-slate-400 opacity-0 group-hover:opacity-100 transition-opacity" />
                    </div>
                    <p className="text-sm text-slate-600 dark:text-slate-400 mt-1">
                      {resource.description}
                    </p>
                  </div>
                </div>
              </motion.a>
            ))}
          </div>
        </motion.div>

        {/* Guides */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-4xl mx-auto mb-16"
        >
          <h2 className="text-2xl font-bold mb-6 text-center text-slate-900 dark:text-white">
            Guides
          </h2>
          <div className="grid md:grid-cols-2 gap-4">
            {guides.map((guide, index) => {
              const Component = guide.internal ? Link : 'a'
              const props = guide.internal
                ? { href: guide.link }
                : { href: guide.link, target: '_blank', rel: 'noopener noreferrer' }

              return (
                <motion.div
                  key={guide.title}
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  transition={{ delay: index * 0.05 }}
                  viewport={{ once: true }}
                >
                  <Component
                    {...props}
                    className="flex items-center gap-4 p-4 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all group"
                  >
                    <div className="w-10 h-10 bg-slate-100 dark:bg-slate-800 rounded-lg flex items-center justify-center flex-shrink-0">
                      <guide.icon className="w-5 h-5 text-slate-600 dark:text-slate-400" />
                    </div>
                    <div className="flex-1">
                      <h3 className="font-semibold text-slate-900 dark:text-white">
                        {guide.title}
                      </h3>
                      <p className="text-sm text-slate-500 dark:text-slate-400">
                        {guide.description}
                      </p>
                    </div>
                    <ArrowRight className="w-5 h-5 text-slate-400 group-hover:text-primary group-hover:translate-x-1 transition-all" />
                  </Component>
                </motion.div>
              )
            })}
          </div>
        </motion.div>

        {/* Quick Example */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-3xl mx-auto mb-16"
        >
          <h2 className="text-2xl font-bold mb-6 text-center text-slate-900 dark:text-white">
            Quick Example
          </h2>
          <div className="rounded-xl border border-slate-700 overflow-hidden">
            <div className="bg-slate-800 px-4 py-3 border-b border-slate-700 flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Code className="w-4 h-4 text-slate-400" />
                <span className="text-sm font-medium text-slate-300">Python</span>
              </div>
              <CopyButton text={`from shodh_memory import Memory

# Initialize
m = Memory()

# Store a memory
m.remember("User prefers dark mode", memory_type="Decision", tags=["preferences"])

# Recall memories
results = m.recall("What are the user preferences?")
for r in results:
    print(f"[{r['experience_type']}] {r['content']}")

# Get context summary for LLM bootstrap
summary = m.context_summary()
print(summary["decisions"])`} />
            </div>
            <div className="bg-slate-900 p-6 overflow-x-auto">
              <pre className="text-sm text-slate-300 font-mono">
{`from shodh_memory import Memory

# Initialize
m = Memory()

# Store a memory
m.remember("User prefers dark mode", memory_type="Decision", tags=["preferences"])

# Recall memories
results = m.recall("What are the user preferences?")
for r in results:
    print(f"[{r['experience_type']}] {r['content']}")

# Get context summary for LLM bootstrap
summary = m.context_summary()
print(summary["decisions"])`}
              </pre>
            </div>
          </div>
        </motion.div>

        {/* Help Section */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="text-center p-12 rounded-2xl bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            Need Help?
          </h2>
          <p className="text-lg text-slate-600 dark:text-slate-400 mb-8 max-w-xl mx-auto">
            Check GitHub issues or reach out directly
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <a
              href="https://github.com/varun29ankuS/shodh-memory/issues"
              target="_blank"
              rel="noopener noreferrer"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
            >
              <Github className="w-5 h-5" />
              GitHub Issues
            </a>
            <a
              href="https://wa.me/919810300618?text=Hi, I need help with Shodh Memory"
              target="_blank"
              rel="noopener noreferrer"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white flex items-center justify-center gap-2"
            >
              <MessageCircle className="w-5 h-5" />
              WhatsApp
            </a>
          </div>
        </motion.div>
      </div>
    </main>
  )
}
